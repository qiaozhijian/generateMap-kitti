//
// Created by qzj on 2020/12/23.
//

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
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

class Mapping {
private:

    ros::NodeHandle nh;

    ros::Publisher map_pub;

    ros::Subscriber subRawScan;
    ros::Subscriber subTransformedScan;

    pcl::PointCloud<PointType>::Ptr rawScan;
    pcl::PointCloud<PointType>::Ptr transformedScan;
    pcl::PointCloud<PointType>::Ptr GlobalMapDS;
    PointType nanPoint;

    pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;

    double timeRawScan;
    double timeTransformedScan;
    double timeLastProcessing;

    bool newRawScan;
    bool newTransformedScan;

    Eigen::Matrix4f T_cur;
    Eigen::Matrix4f T_prev;

public:
    Mapping() :
            nh("~") {

        map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 10);
        subRawScan = nh.subscribe<sensor_msgs::PointCloud2>("/raw_scan", 2, &Mapping::rawScanHandler, this);
        subTransformedScan = nh.subscribe<sensor_msgs::PointCloud2>("/transformed_scan", 2,
                                                                    &Mapping::transformedScanHandler, this);

        allocateMemory();

    }

    void run() {
        if (newRawScan && std::abs(timeRawScan - timeTransformedScan) < 0.005 &&
            newTransformedScan) {

            newRawScan = false;
            newTransformedScan = false;

            if (timeRawScan - timeLastProcessing >= 0.3) {
                timeLastProcessing = timeRawScan;
                *GlobalMapDS += *transformedScan;
                downSizeFilterGlobalMap.setInputCloud(GlobalMapDS);
                downSizeFilterGlobalMap.filter(*GlobalMapDS);
                publishCloud();
            }

        }
    }

    void allocateMemory() {

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();//quiet_NaN：返回目标类型的安静NaN的表示
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();

        downSizeFilterGlobalMap.setLeafSize(0.8, 0.8, 0.8);//设置栅格大小，单位：m

        rawScan.reset(new pcl::PointCloud<PointType>());
        transformedScan.reset(new pcl::PointCloud<PointType>());;
        GlobalMapDS.reset(new pcl::PointCloud<PointType>());

        timeRawScan = 0;
        timeTransformedScan = 0;
        timeLastProcessing = -1;

        newRawScan = false;
        newTransformedScan = false;
    }

    ~Mapping() {}

    void rawScanHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
        timeRawScan = msg->header.stamp.toSec();
        rawScan->clear();
        pcl::fromROSMsg(*msg, *rawScan);
        newRawScan = true;
    }

    void transformedScanHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
        timeTransformedScan = msg->header.stamp.toSec();
        transformedScan->clear();
        pcl::fromROSMsg(*msg, *transformedScan);
        newTransformedScan = true;
    }

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
        pcl::toROSMsg(*GlobalMapDS, output);   // 转换成ROS下的数据类型 最终通过topic发布
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "slam";
        map_pub.publish(output);
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "mapping");

    ROS_INFO("\033[1;32m---->\033[0m mapping Started.");

    Mapping mapping;

    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();

        mapping.run();

        rate.sleep();
    }

    return 0;
}
