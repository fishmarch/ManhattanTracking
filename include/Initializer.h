#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <cmath>
#include <random>
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "Frame.h"


using namespace std;
#define pi 3.1415926

namespace MANHATTAN_TRACKING{

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    class Initializer{
public:
    Initializer(PointCloud::Ptr InitializationPointCloud, int tim, float window, bool UseGaussianCore);
    Initializer(Frame* frame, int tim, float window, bool UseGaussian);

    bool Initialize();
    Eigen::Matrix3f R() { return mR; }

private:
    void RiemannMapping();
    void RiemannUnmapping();
    void MeanShift(float& x_mean, float& y_mean, float (*dis)(float, float, float, float, float));
    float KernelDensityEstimate(float x_mean, float y_mean, float (*dis)(float, float, float, float, float));
    void ClearData();

    //pointcloud for initialization
    PointCloud::Ptr mInitializationPointCloud;
    //pointcloud to save riemann mapping
    PointCloud::Ptr mRiemannPointCloud;
    //pointcloud of meanshift results
    PointCloud::Ptr mMeanShiftPoints;

    //num of initialization time
    const int mTime;
    const float mWindow;
    const bool mUseGaussianCore;

    vector<PointT> mInitPoints;
    float mlam[3];

    Eigen::Matrix3f mR;

    Frame* mFrame;
    
    static float DisUniformCore(float x, float y, float x_mean, float y_mean, float window);
    static float DisGaussianCore(float x, float y, float x_mean, float y_mean, float window);

    };

    } // namespace MANHATTAN_TRACKING

#endif