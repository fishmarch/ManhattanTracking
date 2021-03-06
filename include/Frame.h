#ifndef MANHATTANTRACKING_FRAME_H
#define MANHATTANTRACKING_FRAME_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

namespace MANHATTAN_TRACKING {
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    struct Camera{
        float factor;
        float cx;
        float cy;
        float fx;
        float fy;
        Camera(): fx(0),fy(0),cx(0),cy(0),factor(0){}
    };

    class Frame{
    public:
        Frame(const cv::Mat& depth, const cv::Mat& rgb, const Eigen::Matrix3f R);
        Frame(const cv::Mat& depth, const cv::Mat& rgb); //First Frame

        inline long unsigned int Id();
        inline PointCloud::Ptr RiemannPoints();
        inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud();
        inline PointCloud::Ptr NormalPoints();
        inline vector<PointCloud::Ptr>& RiemannPointCloud();

    private:
        void GenerateCloud(const cv::Mat& depth, const cv::Mat& rgb);
        void NormalExtract();
        void RiemannMapping();
        void FirstRiemannMapping();

        PointCloud::Ptr mRiemannPoints;
        vector<PointCloud::Ptr> mRiemannPointCloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr mCloud;
        PointCloud::Ptr mNormalPoints;

        long unsigned int mId;
        Eigen::Matrix3f mLastR;

        static long unsigned int mLastId;

    public:
        static Camera mCamera;
        static void SetCameraParameter(float fx_, float fy_, float cx_, float cy_, float factor_);
    };// CLASS FRAME

    unsigned long int Frame::Id() {
        return mId;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Frame::Cloud(){
        return mCloud;
    }

    PointCloud::Ptr Frame::NormalPoints(){
        return mNormalPoints;
    }

    PointCloud::Ptr Frame::RiemannPoints(){
        return mRiemannPoints;
    }

    vector<PointCloud::Ptr>& Frame::RiemannPointCloud(){
        return mRiemannPointCloud;
    }

}


#endif //MANHATTANTRACKING_FRAME_H