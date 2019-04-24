//
// Created by fishmarch on 19-4-18.
//
#include "Tracking.h"

namespace MANHATTAN_TRACKING{

    Tracking::Tracking(char* ConfigFile): mState(NO_IMAGES_YET), mConfigFile(ConfigFile){
        cv::FileStorage fs(mConfigFile, cv::FileStorage::READ);
        fs["window_size"] >> mWindow;
        fs["UseGaussianCore"] >> mUseGaussianCore;
    }

    void Tracking::GrabFrame(const cv::Mat &depth, const cv::Mat &rgb) {

        //ClearData();
        if(mState==NO_IMAGES_YET){
            mCurrentFrame = new Frame(depth);
            mState = NOT_INITIALIZED;
        }else{
            mCurrentFrame = new Frame(depth, mLastR);
            mPoints[0] = mLastR.col(0);
            mPoints[1] = mLastR.col(1);
            mPoints[2] = mLastR.col(2);
        }
        Track();
    }


    void Tracking::Track(){

        if(mState==NOT_INITIALIZED){
            cv::FileStorage fs(mConfigFile, cv::FileStorage::READ);
            int trytim;
            fs["initilize_time"] >> trytim;
            mInitializer = new Initializer(mCurrentFrame, trytim, mWindow, mUseGaussianCore);
            PrintString("Iniializing...");
            int init_tim = 10;
            while (init_tim--) {
                if(mInitializer->Initialize()){
                    PrintString("Initialize Successfully");
                    mR = mInitializer->R();
                    cout << mR << endl;
                    mLastR = mR;
                    mState = OK;
                    return;
                }
            }
            PrintString("Initialize Failed. Try Again...");
            return;
        }

        if(TrackRotation()){
            Eigen::Matrix3f Rfail;
            Rfail << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;
            if(mR != Rfail)
                mLastR = mR;
            mState = OK;
            //TODO: TrackTranslation();
        }
    }

    bool Tracking::TrackRotation() {
        PrintString("Tracking Rotation...");

        RiemannMapping();
        cout << mPoints[0].transpose() << endl;
        for (int i = 0; i < 3; ++i) {
            if(mUseGaussianCore)
                MeanShift(i, mPoints[i](0), mPoints[i](1), DisGaussianCore);
            else
                MeanShift(i, mPoints[i](0), mPoints[i](1), DisUniformCore);

            float p1 = KernelDensityEstimate(i, mPoints[i](0), mPoints[i](1), DisUniformCore);
            float p2 = KernelDensityEstimate(i, mPoints[i](0), mPoints[i](1), DisGaussianCore);
            float lam = p2 / p1;

            mlam[i] = lam;
        }
        cout << mPoints[0].transpose() << endl;

        RiemannUnmapping();

        cout << mPoints[0].transpose() << endl;

        Eigen::Vector3f v1; v1 << mPoints[0](0), mPoints[0](1), mPoints[0](2);
        Eigen::Vector3f v2; v2 << mPoints[1](0), mPoints[1](1), mPoints[1](2);
        Eigen::Vector3f v3; v3 << mPoints[2](0), mPoints[2](1), mPoints[2](2);

        v1 *= mlam[0]; v2 *= mlam[1]; v3 *= mlam[2];

        mR << v1, v2, v3;
        cout << mR << endl;
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(mR, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::Matrix3f left_singular_vectors = svd.matrixU();
        Eigen::Matrix3f right_singular_vectors = svd.matrixV();
        mR = left_singular_vectors*right_singular_vectors.transpose();
        cout << mR << endl;
        return true;
    }

    void Tracking::RiemannMapping() {
        for(int i = 0; i < 3; ++i){
            Eigen::Vector3f& p = mPoints[i];

            float lam = sqrt(p(0)*p(0) + p(1)*p(1));
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
        for(int i = 0; i < 3; ++i){
            Eigen::Vector3f& p = mPoints[i];
            float s = p(0)*p(0) + p(1)*p(1);
            s = sqrt(s);
            float th = tan(s);
            p(0) = th*p(0)/s;
            p(1) = th*p(1)/s;
            p(2) = 1;
            float norm = p.norm();
            p(0) = -p(0)/norm;
            p(1) = -p(1)/norm;
            p(2) = -p(2)/norm;
        }
    }

    void Tracking::MeanShift(int id, float& x_mean, float& y_mean, float (*dis)(float, float, float, float, float)){
        float x_sum = 0;
        float y_sum = 0;
        float weights = 0;
        float x_diff = 0;
        float y_diff = 0;
        float diff = 1;

        while (diff > 0.0001) {
            weights = 0;
            x_sum = 0;
            y_sum = 0;
            for (int i = 0; i < mCurrentFrame->mRiemannPointCloud[id]->points.size(); ++i) {
                PointT n = mCurrentFrame->mRiemannPointCloud[id]->points[i];
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
            diff = pow(x_diff, 2) + pow(y_diff, 2);
        }
    }

    void Tracking::ClearData() {
//        if(mInitializer)
//            delete mInitializer;
//        if(mCurrentFrame)
//            delete mCurrentFrame;
    }


    float Tracking::KernelDensityEstimate(int id, float x_mean, float y_mean, float (*dis)(float, float, float, float, float)){
        float weights = 0;
        for (int i = 0; i < mCurrentFrame->mRiemannPointCloud[id]->points.size(); ++i) {
            PointT n = mCurrentFrame->mRiemannPointCloud[id]->points[i];
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
        return res;
    }

}// MANHATTAN_TRACKING
