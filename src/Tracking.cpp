//
// Created by fishmarch on 19-4-18.
//
#include "Tracking.h"

namespace MANHATTAN_TRACKING{

    Tracking::Tracking(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TrackingPointCLoud,
                       Eigen::Matrix3f LastR, float Window, bool UseGaussianCore):
                       mTrackingPointCloud(TrackingPointCLoud),
                       mWindow(Window), mUseGaussianCore(UseGaussianCore){
        cout << "Tracking making ..." << endl;
        Eigen::Vector3f R1 = LastR.col(0);
        Eigen::Vector3f R2 = LastR.col(1);
        Eigen::Vector3f R3 = LastR.col(2);
        mPoints.push_back(R1);
        mPoints.push_back(R2);
        mPoints.push_back(R3);
        PointCloud::Ptr cloud1(new PointCloud);
        PointCloud::Ptr cloud2(new PointCloud);
        PointCloud::Ptr cloud3(new PointCloud);
        mRiemannPointCloud.push_back(cloud1);
        mRiemannPointCloud.push_back(cloud2);
        mRiemannPointCloud.push_back(cloud3);
        RiemannMapping();
        cout << "Tracking made" << endl;
    }

    bool Tracking::Track(){
        TrackRotation();
        //TODO: TrackTranslation();
        return true;
    }

    bool Tracking::TrackRotation() {
        cout << "Track Rotation..." << endl;
        for (int i = 0; i < 3; ++i) {
            if(mUseGaussianCore)
                MeanShift(i, mPoints[i](0), mPoints[i](1),DisGaussianCore);
            else
                MeanShift(i, mPoints[i](0), mPoints[i](1),DisUniformCore);

            float p1 = KernelDensityEstimate(i, mPoints[i](0), mPoints[i](0), DisUniformCore);
            float p2 = KernelDensityEstimate(i, mPoints[i](0), mPoints[i](0), DisGaussianCore);
            float lam = p2 / p1;

            mlam.push_back(lam);
        }

        RiemannUnmapping();
        
        Eigen::Vector3f v1; v1 << mPoints[0](0), mPoints[0](1), mPoints[0](2);
        Eigen::Vector3f v2; v2 << mPoints[1](0), mPoints[1](1), mPoints[1](2);
        Eigen::Vector3f v3; v3 << mPoints[2](0), mPoints[2](1), mPoints[2](2);

        v1 *= mlam[0]; v2 *= mlam[1]; v3 *= mlam[2];

        mR << v1, v2, v3;
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(mR, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::Matrix3f left_singular_vectors = svd.matrixU();
        Eigen::Matrix3f right_singular_vectors = svd.matrixV();
        mR = left_singular_vectors*right_singular_vectors.transpose();

        return true;
    }

    void Tracking::RiemannMapping(){
        cout << "Riemann mapping..." << endl;
        for (int i = 0; i < mTrackingPointCloud->points.size(); ++i) {
            PointT n = mTrackingPointCloud->points[i];
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
                float temp = mPoints[j].dot(vn);
                if(temp > sqrt(0.5)){
                    mRiemannPointCloud[j]->points.push_back(p);
                    break;
                }
            }
        }
        for(auto& p:mPoints){
            float lam = sqrt(p(0) * p(0) + p(1) * p(1));
            float theta = asin(lam);
            p(0) = theta * p(0) / lam;
            p(1) = theta * p(1) / lam;
            if(p(2) < 0){
                p(0) = -p(0);
                p(1) = -p(1);
            }
            p(2) = 1;
        }

    }

    void Tracking::RiemannUnmapping(){
        for(auto& p : mPoints){
            float s = p(0)*p(0) + p(1)*p(1);
            s = sqrt(s);
            float th = tan(s);
            p(0) = th*p(0)/s;
            p(1) = th*p(1)/s;
            p(2) = 1;

            p(0) = -p(0)/p.norm();
            p(1) = -p(1)/p.norm();
            p(2) = -p(2)/p.norm();
        }
    }

    float Tracking::MeanShift(int id, float& x_mean, float& y_mean, float (*dis)(float, float, float, float, float)){
        float x_sum = 0;
        float y_sum = 0;
        float weights = 0;
        float x_diff = 1;
        float y_diff = 1;
        float diff = 1;

        while (diff > 0.0001) {
            weights = 0;
            x_sum = 0;
            y_sum = 0;
            for (int i = 0; i < mRiemannPointCloud[id]->points.size(); ++i) {
                PointT n = mRiemannPointCloud[id]->points[i];
                if (isnan(n.x))
                    continue;
                float x_dis = n.x - x_mean;
                float y_dis = n.y - y_mean;
                float weight = dis(n.x, n.y, x_mean, y_mean, mWindow);
                x_sum += x_dis * weight;
                y_sum += y_dis * weight;
                weights += weight;
            }
            x_diff = x_sum / weights;
            y_diff = y_sum / weights;
            x_mean += x_diff;
            y_mean += y_diff;
            //cout << "move  point: " << x_mean << " , " << y_mean << " num: " << num << endl;
            diff = pow(x_diff, 2) + pow(y_diff, 2);
        }
        return weights;
    }

    float Tracking::KernelDensityEstimate(int id, float x_mean, float y_mean, float (*dis)(float, float, float, float, float)){
        float weights = 0;
        for (int i = 0; i < mRiemannPointCloud[id]->points.size(); ++i) {
            PointT n = mRiemannPointCloud[id]->points[i];
            if (isnan(n.x))
                continue;
            float weight = dis(n.x, n.y, x_mean, y_mean, mWindow);
            weights += weight;
        }
        return weights;
    }

    float Tracking::DisUniformCore(float x, float y, float x_mean, float y_mean, float window){
        float x_dis = x - x_mean;
        float y_dis = y - y_mean;
        if (pow(x_dis, 2) + pow(y_dis, 2) > window*window)
            return 0;
        else
            return 1;
    }

    float Tracking::DisGaussianCore(float x, float y, float x_mean, float y_mean, float window){
        float x_dis = x - x_mean;
        float y_dis = y - y_mean;
        float dis = pow(x_dis, 2) + pow(y_dis, 2);
        float sigma = window / 3;

        float res = exp(-dis / (2 * sigma * sigma));
        //res = res / (sqrt(2 * pi) * sigma);
        return res;
    }

}// MANHATTAN_TRACKING
