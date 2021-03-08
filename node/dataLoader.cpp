#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include "ros/ros.h"
#include <string>
#include "../include/loadData.h"
#include <vector>
#include "sensor_msgs/PointCloud2.h"
#include "../include/selfDefine.h"

using namespace std;
using namespace Eigen;

class DataLoader {
private:

    ros::NodeHandle nh;

    ros::Publisher pubRawScan;
    ros::Publisher pubTransformedScan;

    pcl::PointCloud<PointType>::Ptr rawScan;
    pcl::PointCloud<PointType>::Ptr rawScanDS;
    pcl::PointCloud<PointType>::Ptr transformedScanDS;
    PointType nanPoint;

    pcl::VoxelGrid<PointType> downSizeFilter;
    pcl::PassThrough<PointType> passThroughFilter;     //创建滤波器对象

    string path_to_bin = "";
    string path_to_pose = "";
    string seq = "";

    vector<string> pcl_paths;
    vector<Eigen::Matrix4f> poses;

    Eigen::Matrix4f T_cur;
    Eigen::Matrix4f T_prev;
    Eigen::Matrix4f Velocity;

public:
    DataLoader() :
            nh("~") {

        //注意添加/，表示ros空间里的变量
        nh.param<std::string>("path_to_bin", path_to_bin, "");
        nh.param<std::string>("path_to_pose", path_to_pose, "0");
        nh.param<std::string>("seq", seq, "0");

        pubRawScan = nh.advertise<sensor_msgs::PointCloud2>("/raw_scan", 10);
        pubTransformedScan = nh.advertise<sensor_msgs::PointCloud2>("/transformed_scan", 10);

        allocateMemory();

        int size = poses.size();
        ros::Rate rate(10);
        for (int idx = 0; idx < size; idx++) {
            readKittiPclBinData(pcl_paths[idx], rawScan);

            downSizeFilter.setInputCloud(rawScan);
            downSizeFilter.filter(*rawScanDS);

            if (idx > 0) {
                T_cur = toVelodyneCoord(poses[idx]);
                Velocity = relativeTrans(T_prev, T_cur);
                T_prev = T_cur;
            }

            transformPointCloud(rawScanDS, transformedScanDS, T_cur);

            passThroughFilter.setInputCloud(transformedScanDS);                //设置待滤波的点云
            passThroughFilter.filter(*transformedScanDS);               //滤波并存储

            publishCloud();

            clearCloud();

            rate.sleep();
        }
    }

    void allocateMemory() {

        getSortedPCL(getPathPCL(path_to_bin, seq), pcl_paths);
        loadPoses(getPathPose(path_to_pose, seq), poses);
        assert(poses.size() == pcl_paths.size());

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();//quiet_NaN：返回目标类型的安静NaN的表示
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();

        downSizeFilter.setLeafSize(0.4, 0.4, 0.4);//设置栅格大小，单位：m

        passThroughFilter.setFilterFieldName("z");             //设置在Z轴方向上进行滤波
        passThroughFilter.setFilterLimits(-2.f, 1000.f);
        passThroughFilter.setFilterLimitsNegative(false);      //保留

        rawScan.reset(new pcl::PointCloud<PointType>());
        rawScanDS.reset(new pcl::PointCloud<PointType>());;
        transformedScanDS.reset(new pcl::PointCloud<PointType>());;

        T_cur = toVelodyneCoord(poses[0]);
        T_prev = T_cur;
        Velocity = relativeTrans(T_prev, T_cur);

        ROS_INFO("allocateMemory finish");
    }

    ~DataLoader() {}

    pcl::PointCloud<PointType>::Ptr
    transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut,
                        Eigen::Matrix4f transformIn) {

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->clear();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i) {
            pointFrom = &cloudIn->points[i];
            pointTo.x = transformIn(0, 0) * pointFrom->x + transformIn(0, 1) * pointFrom->y +
                        transformIn(0, 2) * pointFrom->z + transformIn(0, 3);
            pointTo.y = transformIn(1, 0) * pointFrom->x + transformIn(1, 1) * pointFrom->y +
                        transformIn(1, 2) * pointFrom->z + transformIn(1, 3);
            pointTo.z = transformIn(2, 0) * pointFrom->x + transformIn(2, 1) * pointFrom->y +
                        transformIn(2, 2) * pointFrom->z + transformIn(2, 3);
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }

        return cloudOut;
    }

    void publishCloud() {

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*rawScan, output);   // 转换成ROS下的数据类型 最终通过topic发布
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "slam";
        pubRawScan.publish(output);

        pcl::toROSMsg(*transformedScanDS, output);   // 转换成ROS下的数据类型 最终通过topic发布
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "slam";
        pubTransformedScan.publish(output);

    }

    void clearCloud() {

        rawScanDS->clear();
        transformedScanDS->clear();

    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "dataLoader");

    ROS_INFO("\033[1;32m---->\033[0m dataLoader Started.");

    DataLoader dataloader;

    ros::spin();

    return 0;
}



