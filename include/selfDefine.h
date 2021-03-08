//
// Created by qzj on 2020/12/22.
//

#ifndef SRC_SELFDEFINE_H
#define SRC_SELFDEFINE_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "Eigen/Core"
#include <string>

typedef pcl::PointXYZI PointType;
extern const std::string pointCloudTopic = "/velodyne_points";
extern const std::string imuTopic = "/imu/data";

extern const float nearSubMapSearchRadius = 25.0;

#endif //SRC_SELFDEFINE_H
