#ifndef LOAD_DATA_H
#define LOAD_DATA_H

#include <string>
#include <vector>
#include "selfDefine.h"

void readKittiPclBinData(std::string &in_file, pcl::PointCloud<PointType>::Ptr cloud);

std::string getPathPCL(const std::string &pathP, const std::string pathC);

std::string getPathPose(const std::string &pathP, const std::string pathC);

void getSortedPCL(const std::string pcl_path, std::vector<std::string> &pcl_paths);

void loadPoses(std::string file_name, std::vector<Eigen::Matrix4f> &poses);

Eigen::Matrix4f toVelodyneCoord(Eigen::Matrix4f pose_cam);

Eigen::Matrix4f relativeTrans(Eigen::Matrix4f prev, Eigen::Matrix4f cur);


#endif // MATRIX_H
