#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <pcl/filters/crop_box.h>
#include "../include/loadData.h"
#include <vector>
#include "../include/selfDefine.h"

using namespace std;
using namespace Eigen;

class DataLoader {
private:

    pcl::PointCloud<PointType>::Ptr rawScan;
    pcl::PointCloud<PointType>::Ptr rawScanDS;
    pcl::PointCloud<PointType>::Ptr transformedScanDS;
    pcl::PointCloud<PointType>::Ptr GlobalMapDS;
    PointType nanPoint;

    pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;
    pcl::VoxelGrid<PointType> downSizeFilter;
    pcl::PassThrough<PointType> passThroughFilter;     //创建滤波器对象

    string path_to_bin = "/media/qzj/Dataset/slamDataSet/kitti/odometry/data_odometry_velodyne/dataset/sequences/";
    string path_to_pose = "/media/qzj/Dataset/slamDataSet/kitti/odometry/data_odometry_velodyne/dataset/poses/";
    string seq = "00";

    vector<string> pcl_paths;
    vector<Eigen::Matrix4d> poses;

    Eigen::Matrix4d T_cur;
    Eigen::Matrix4d Velocity;

public:
    DataLoader() {

        allocateMemory();

        int size = poses.size();
        for (int idx = 0; idx < size; idx++) {
            readKittiPclBinData(pcl_paths[idx], rawScan);

            downSizeFilter.setInputCloud(rawScan);
            downSizeFilter.filter(*rawScanDS);

            T_cur = toVelodyneCoord(poses[idx]);

            pcl::transformPointCloud(*rawScanDS, *transformedScanDS, T_cur);
//            pcl::io::savePCDFileASCII("./frame_"+to_string(idx)+".pcd", *transformedScanDS);

            passThroughFilter.setInputCloud(transformedScanDS);                //设置待滤波的点云
            passThroughFilter.filter(*transformedScanDS);               //滤波并存储

            *GlobalMapDS += *transformedScanDS;

            if(idx % 40 == 0){
                printf("process %d\n", idx);
                FilterGlobalAreas(downSizeFilterGlobalMap, GlobalMapDS, GlobalMapDS);
            }
            clearCloud();
        }

        pcl::io::savePCDFileASCII("./"+seq+"_map.pcd", *GlobalMapDS);

    }

    Eigen::Affine3f ICPRegistration(pcl::PointCloud<PointType>::Ptr source, pcl::PointCloud<PointType>::Ptr target){
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(1);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(source);
        icp.setInputTarget(target);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        Eigen::Affine3f result;
        result = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)

        return result;
    }

    void FilterGlobalAreas(pcl::VoxelGrid<PointType> &filter
            , const pcl::PointCloud<PointType>::Ptr & cloud, pcl::PointCloud<PointType>::Ptr & cloudDS){
        PointType min;	//xyz的最小值
        PointType max;	//xyz的最大值
        pcl::getMinMax3D(*cloud,min,max);	//获取所有点中的坐标最值

        const double cube_length = 50;
        int nx = (max.x - min.x) / cube_length + 1;
        int ny = (max.y - min.y) / cube_length + 1;
        int nz = (max.z - min.z) / cube_length + 1;

        cloudDS->clear();

        pcl::CropBox<PointType> pcl_box_filter_;
        pcl_box_filter_.setInputCloud(cloud);
        std::vector<pcl::PointCloud<PointType>::Ptr> pointAreas;
        pointAreas.resize(nx*ny*nz);
        for(int x=0; x<nx; x++)
            for(int y=0; y<ny; y++)
                for(int z=0; z<nz; z++) {
                    uint32_t index = x * ny * nz + y * nz + z;
                    pointAreas[index].reset(new pcl::PointCloud<PointType>());
                    pcl_box_filter_.setMin(Eigen::Vector4f(min.x + x * cube_length, min.y + y * cube_length,
                                                           min.z + z * cube_length,1.0));
                    pcl_box_filter_.setMax(Eigen::Vector4f(min.x + (x + 1) * cube_length, min.y + (y + 1) * cube_length,
                                                           min.z + (z + 1) * cube_length, 1.0));
                    pcl_box_filter_.filter(*pointAreas[index]);
                    filter.setInputCloud(pointAreas[index]);
                    filter.filter(*pointAreas[index]);
                    *cloudDS += *pointAreas[index];
                }
    }

    void allocateMemory() {

        getSortedPCL(getPathPCL(path_to_bin, seq), pcl_paths);
        loadPoses(getPathPose(path_to_pose, seq), poses);
        assert(poses.size() == pcl_paths.size());

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();//quiet_NaN：返回目标类型的安静NaN的表示
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();

        downSizeFilter.setLeafSize(0.1, 0.1, 0.1);//设置栅格大小，单位：m
        downSizeFilterGlobalMap.setLeafSize(0.1, 0.1, 0.1);//设置栅格大小，单位：m

        passThroughFilter.setFilterFieldName("z");             //设置在Z轴方向上进行滤波
        passThroughFilter.setFilterLimits(-2.f, 1000.f);
        passThroughFilter.setFilterLimitsNegative(false);      //保留

        rawScan.reset(new pcl::PointCloud<PointType>());
        rawScanDS.reset(new pcl::PointCloud<PointType>());;
        transformedScanDS.reset(new pcl::PointCloud<PointType>());;
        GlobalMapDS.reset(new pcl::PointCloud<PointType>());

        T_cur = toVelodyneCoord(poses[0]);
    }

    ~DataLoader() {}

    void clearCloud() {

        rawScanDS->clear();
        transformedScanDS->clear();

    }
};

int main(int argc, char **argv) {

    DataLoader dataloader;

    return 0;
}



