//
// Created by fishmarch on 19-4-18.
//

#ifndef MANHATTANTRACKING_TRACKING_H
#define MANHATTANTRACKING_TRACKING_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

namespace MANHATTAN_TRACKING{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    class Tracking{
    public:
        Tracking(PointCloud::Ptr TrackingPointCLoud,Eigen::Matrix3f LastR, float Window, bool UseGaussianCore,
                pcl::visualization::PCLVisualizer& viewer, int& port);
        bool Track();
        Eigen::Matrix3f R12() { return mR; }
    private:
        void RiemannMapping();
        void RiemannUnmapping();
        bool TrackRotation();
        float MeanShift(int id, float& x_mean, float& y_mean, float (*dis)(float, float, float, float, float));
        float KernelDensityEstimate(int id, float x_mean, float y_mean, float (*dis)(float, float, float, float, float));

        PointCloud::Ptr mTrackingPointCloud;
        //pointcloud to save riemann mapping
        vector<PointCloud::Ptr> mRiemannPointCloud;

        //pointcloud of meanshift results
        PointCloud::Ptr mMeanShiftPoints;

        //for pcl_viewer
        pcl::visualization::PCLVisualizer mViewer;
        int mPort;

        const float mWindow;
        const bool mUseGaussianCore;

        //Eigen::Matrix3f mLastR;
        vector<Eigen::Vector3f> mPoints;
        vector<float> mlam;

        Eigen::Matrix3f mR;
        static float DisUniformCore(float x, float y, float x_mean, float y_mean, float window);
        static float DisGaussianCore(float x, float y, float x_mean, float y_mean, float window);
    }; // CLASS Tracking
} // MANHATTAN_TRACKING

#endif //MANHATTANTRACKING_TRACKING_H
