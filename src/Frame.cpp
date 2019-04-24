#include "Frame.h"

namespace MANHATTAN_TRACKING{

    long unsigned int Frame::mLastId = 0;
    Camera Frame::mCamera = Camera();

    Frame::Frame(const cv::Mat &depth, const cv::Mat& rgb, const Eigen::Matrix3f R):
        mCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        mNormalPoints(new PointCloud) {
        GenerateCloud(depth, rgb);
        NormalExtract();
        mLastR = R;

        PointCloud::Ptr cloud1(new PointCloud);
        PointCloud::Ptr cloud2(new PointCloud);
        PointCloud::Ptr cloud3(new PointCloud);
        mRiemannPointCloud.push_back(cloud1);
        mRiemannPointCloud.push_back(cloud2);
        mRiemannPointCloud.push_back(cloud3);

        RiemannMapping();

        mId = ++mLastId;
    }

    Frame::Frame(const cv::Mat &depth, const cv::Mat& rgb):
        mCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        mNormalPoints(new PointCloud),
        mRiemannPoints(new PointCloud){
        GenerateCloud(depth, rgb);
        NormalExtract();
        FirstRiemannMapping();
        mId = ++mLastId;
    }


    void Frame::GenerateCloud(const cv::Mat &depth, const cv::Mat& rgb) {
        for (int m = 0; m < depth.rows; m++){
            for (int n=0; n < depth.cols; n++){
                ushort d = depth.ptr<ushort>(m)[n];
                pcl::PointXYZRGB p;
                p.z = float(d) / mCamera.factor;
                p.x = (n - mCamera.cx) * p.z / mCamera.fx;
                p.y = (m - mCamera.cy) * p.z / mCamera.fy;
                p.b = rgb.ptr<uchar>(m)[n*3];
                p.g = rgb.ptr<uchar>(m)[n*3+1];
                p.r = rgb.ptr<uchar>(m)[n*3+2];
                mCloud->points.push_back( p );
            }
        }
        mCloud->height = depth.rows;
        mCloud->width = depth.cols;
    }

    void Frame::NormalExtract() {
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(mCloud);
        ne.compute(*cloud_normals);

        mNormalPoints->width = cloud_normals->points.size();
        mNormalPoints->height = 1;
        mNormalPoints->points.resize(mNormalPoints->width * mNormalPoints->height);
        for (int j = 0; j < cloud_normals->points.size(); ++j) {
            mNormalPoints->points[j].x = cloud_normals->points[j].normal_x;
            mNormalPoints->points[j].y = cloud_normals->points[j].normal_y;
            mNormalPoints->points[j].z = cloud_normals->points[j].normal_z;
        }
        mNormalPoints->is_dense = false;
    }

    void Frame::FirstRiemannMapping() {
        for (int i = 0; i < mNormalPoints->points.size(); ++i) {
            PointT n = mNormalPoints->points[i];
            if(isnan(n.x))
                continue;

            PointT p;
            float lam = sqrt(n.x * n.x + n.y * n.y);
            float theta = asin(lam);
            p.x = theta * n.x / lam;
            p.y = theta * n.y / lam;
            if(n.z < 0){
                p.x = -p.x;
                p.y = -p.y;
            }
            p.z = 1;

            mRiemannPoints->points.push_back(p);
        }
    }

    void Frame::RiemannMapping() {
        for (int i = 0; i < mNormalPoints->points.size(); ++i) {
            PointT n = mNormalPoints->points[i];
            if(isnan(n.x))
                continue;

            PointT p;
            float lam = sqrt(n.x * n.x + n.y * n.y);
            float theta = asin(lam);
            p.x = theta * n.x / lam;
            p.y = theta * n.y / lam;
            if(n.z < 0){
                p.x = -p.x;
                p.y = -p.y;
            }
            p.z = 1;

            Eigen::Vector3f vn;
            vn << n.x, n.y, n.z;

            //assign the point to respective cone
            for (int j = 0; j < 3; ++j) {
                Eigen::Vector3f vm = mLastR.col(j);
                float temp = vm.dot(vn);
                if(temp > sqrt(0.5)){
                    mRiemannPointCloud[j]->points.push_back(p);
                    break;
                }
            }
        }
    }

    void Frame::SetCameraParameter(float fx_, float fy_, float cx_, float cy_, float factor_) {
        mCamera.cx = cx_;
        mCamera.cy = cy_;
        mCamera.fx = fx_;
        mCamera.fy = fy_;
        mCamera.factor = factor_;
    }
}