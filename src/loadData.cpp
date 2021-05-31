//
// Created by qzj on 2020/12/22.
//
#include "../include/loadData.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/regex.hpp> //Note: using boost regex instead of C++11 regex as it isn't supported by the compiler until gcc 4.9
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

using namespace std;
using namespace Eigen;

string getPathPCL(const std::string &pathP, const std::string pathC) {
    string path = pathP;
    unsigned int iSize = path.size();
    if (path.at(iSize - 1) != '/')
        path.push_back('/');

    path = path + pathC;
    iSize = path.size();
    if (path.at(iSize - 1) != '/')
        path.push_back('/');

    path = path + "velodyne";
    iSize = path.size();
    if (path.at(iSize - 1) != '/')
        path.push_back('/');

    return path;
}

string getPathPose(const std::string &pathP, const std::string pathC) {
    string path = pathP;
    unsigned int iSize = path.size();
    if (path.at(iSize - 1) != '/')
        path.push_back('/');

    path = path + pathC + ".txt";

    return path;
}

void readKittiPclBinData(std::string &in_file, pcl::PointCloud<PointType>::Ptr cloud) {
    cloud->clear();
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        PointType point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud->push_back(point);
    }
    input.close();
}

Eigen::Matrix4d toVelodyneCoord(Eigen::Matrix4d pose_cam) {
    Eigen::Matrix4d T_c_v = Eigen::Matrix4d::Identity();
//    T_c_v << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02, \
//    -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02, \
//    9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01, \
//    0, 0, 0, 1;
    T_c_v << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03, \
    1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02, \
    9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01, \
    0, 0, 0, 1;
    Eigen::Matrix4d pose_vel = T_c_v.inverse() * pose_cam * T_c_v;
    return pose_vel;
}

Eigen::Matrix4d relativeTrans(Eigen::Matrix4d prev, Eigen::Matrix4d cur) {
    return cur * prev.inverse();
}

void loadPoses(string file_name, vector<Eigen::Matrix4d> &poses) {

//    计算有多少个位姿
    FILE *fp_ = fopen(file_name.c_str(), "r");
    if (!fp_) {
        cout << file_name << "wrong path" << endl;
        assert(0);
    }
    int size = 0;
    while (!feof(fp_)) {
        float val[12] = {0.0};
        if (fscanf(fp_, "%f %f %f %f %f %f %f %f %f %f %f %f",
                   val + 0, val + 1, val + 2, val + 3,
                   val + 4, val + 5, val + 6, val + 7,
                   val + 8, val + 9, val + 10, val + 11) == 12) {
            size++;
        }
    }
    fclose(fp_);

    poses.reserve(size);

    FILE *fp = fopen(file_name.c_str(), "r");
    int i, j = 0;
    while (!feof(fp)) {
        Matrix4d P = Eigen::Matrix4d::Identity();
        float val[12] = {0.f};
        if (fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f",
                   val + 0, val + 1, val + 2, val + 3,
                   val + 4, val + 5, val + 6, val + 7,
                   val + 8, val + 9, val + 10, val + 11) == 12) {
            for (i = 0; i < 3; i++)
                for (j = 0; j < 4; j++)
                    P(i, j) = val[i * 4 + j];
            poses.push_back(P);
        }
    }
    fclose(fp);
}

void getSortedPCL(const std::string pcl_path, std::vector<std::string> &pcl_paths) {

    boost::filesystem::path pcl_dir(pcl_path);
    if (!boost::filesystem::exists(pcl_dir) ||
        !boost::filesystem::is_directory(pcl_dir))
        throw std::runtime_error("[Dataset] Invalid directory");

    //https://blog.csdn.net/stephen_yin/article/details/6731545
    boost::regex expression("^[^0-9]*([0-9]*\\.?+[0-9]*)[^0-9]*\\.[a-z]{3,4}$");
    boost::cmatch what;
    auto filename_filter = [&expression, &what](const std::string &s) {
        return !boost::regex_match(s.c_str(), what, expression);
    };

    auto sort_by_number = [&expression, &what](const std::string &a, const std::string &b) {
        double n1, n2;

        if (boost::regex_match(a.c_str(), what, expression))
            n1 = std::stod(what[1]);
        else
            throw std::runtime_error("[Dataset] Unexpected behaviour while sorting filenames");

        if (boost::regex_match(b.c_str(), what, expression))
            n2 = std::stod(what[1]);
        else
            throw std::runtime_error("[Dataset] Unexpected behaviour while sorting filenames");

        return (n1 < n2);
    };

    // get a sorted list of files in the pcl directories
    if (!boost::filesystem::exists(pcl_dir) ||
        !boost::filesystem::is_directory(pcl_dir))
        throw runtime_error("[Dataset] Invalid pcls subfolder");

    // get all files in the pcl directories
    list<string> all_pcls;
    for (auto &entry : boost::make_iterator_range(boost::filesystem::directory_iterator(pcl_dir), {})) {
        boost::filesystem::path filename_path = entry.path().filename();
        if (boost::filesystem::is_regular_file(entry.status()) &&
            (filename_path.extension() == ".bin")) {
            all_pcls.push_back(filename_path.string());
        }
    }

    // sort
    pcl_paths.clear();
    pcl_paths.reserve(all_pcls.size());
    for (const string &filename : all_pcls)
        if (!filename_filter(filename)) pcl_paths.push_back(filename);

    if (pcl_paths.empty()) {
        cout << "pcl_path: " << pcl_path << endl;
        throw runtime_error("[Dataset] Invalid pcl names?");
    }

    sort(pcl_paths.begin(), pcl_paths.end(), sort_by_number);

    for (string &filename : pcl_paths)
        filename = (pcl_dir / filename).string();
}

