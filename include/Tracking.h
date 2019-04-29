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
#include "Initializer.h"
#include "Frame.h"
#include "base.h"


using namespace std;

namespace MANHATTAN_TRACKING{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
        // Tracking states
        enum eTrackingState{
            SYSTEM_NOT_READY=-1,
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,
            OK=2,
            LOST=3
        };
        enum eManhattanState{
            NoLack = 0,
            LackX = 1,
            LackY = 2,
            LackZ = 3
        };


    class Tracking{
    public:

        Tracking(char* ConfigFile);
        bool GrabFrame(const cv::Mat& depth, const cv::Mat& rgb);
        Eigen::Matrix3f R() { return mR; }
        eTrackingState mState;
        eManhattanState mManhattanState;
        Frame* CurrentFrame(){return mCurrentFrame;}
    private:
        void RiemannMapping();
        void RiemannUnmapping();
        bool TrackRotation();
        void MeanShift(int id, float& x_mean, float& y_mean, float (*dis)(float, float, float, float, float));
        float KernelDensityEstimate(int id, float x_mean, float y_mean, float (*dis)(float, float, float, float, float));
        void ClearData();
        bool Track();

        float mWindow;
        bool mUseGaussianCore;

        Eigen::Vector3f mPoints[3];
        float mlam[3];

        Eigen::Matrix3f mR;
        Eigen::Matrix3f mLastR;
        Eigen::Matrix3f mFailR;

        Initializer* mInitializer;
        Frame* mCurrentFrame;
        Frame* mLastFrame;

        char* mConfigFile;

        static float DisUniformCore(float x, float y, float x_mean, float y_mean, float window);
        static float DisGaussianCore(float x, float y, float x_mean, float y_mean, float window);
    }; // CLASS TRACKING
} // MANHATTAN_TRACKING

#endif //MANHATTANTRACKING_TRACKING_H
