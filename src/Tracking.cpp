//
// Created by fishmarch on 19-4-18.
//
#include "Tracking.h"

namespace MANHATTAN_TRACKING{

    Tracking::Tracking(char* ConfigFile): mState(NO_IMAGES_YET), mConfigFile(ConfigFile){
        cv::FileStorage fs(mConfigFile, cv::FileStorage::READ);
        fs["window_size"] >> mWindow;
        fs["UseGaussianCore"] >> mUseGaussianCore;
        mFailR << 1, 0, 0,
                  0, 1, 0,
                  0, 0, 1;
    }

    bool Tracking::GrabFrame(const cv::Mat &depth, const cv::Mat &rgb) {

        //ClearData();
        if(mState==NO_IMAGES_YET){
            mCurrentFrame = new Frame(depth, rgb);
            mState = NOT_INITIALIZED;
        }else{
            delete mCurrentFrame;
            mCurrentFrame = NULL;
            mCurrentFrame = new Frame(depth, rgb, mLastR);
            mPoints[0] = mLastR.col(0);
            mPoints[1] = mLastR.col(1);
            mPoints[2] = mLastR.col(2);
        }
        return Track();
    }


    bool Tracking::Track(){
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
                    return true;
                }
            }
            PrintString("Initialize Failed. Try Again...");
            return false;
        }

        if(TrackRotation()){
            if(mR != mFailR){
                mLastR = mR;
                mState = OK;
                return true;
            }else{
                return false;
            }
            //TODO: TrackTranslation();
        }else{
            PrintString("Tracking Failed");
            cout << "Manhattan State: " << mManhattanState << endl;
            cout << "Failed Frame: " << mCurrentFrame->Id() << endl;
            return false;
        }
    }

    bool Tracking::TrackRotation() {
        PrintString("Tracking Rotation...");

        RiemannMapping();
        mManhattanState = NoLack;

        for (int i = 0; i < 3; ++i) {
            if(mUseGaussianCore)
                MeanShift(i, mPoints[i](0), mPoints[i](1), DisGaussianCore);
            else
                MeanShift(i, mPoints[i](0), mPoints[i](1), DisUniformCore);

            float p1 = KernelDensityEstimate(i, mPoints[i](0), mPoints[i](1), DisUniformCore);
            float p2 = KernelDensityEstimate(i, mPoints[i](0), mPoints[i](1), DisGaussianCore);
            float lam = p2 / p1;

            if(p1 < 5000){
                if(mManhattanState != NoLack)
                    return false;
                switch (i){
                    case 0:
                        mManhattanState = LackX; break;
                    case 1:
                        mManhattanState = LackY; break;
                    case 2:
                        mManhattanState = LackZ; break;
                }
            }
            mlam[i] = lam;
        }

        RiemannUnmapping();

        Eigen::Vector3f v1; v1 << mPoints[0](0), mPoints[0](1), mPoints[0](2);
        Eigen::Vector3f v2; v2 << mPoints[1](0), mPoints[1](1), mPoints[1](2);
        Eigen::Vector3f v3; v3 << mPoints[2](0), mPoints[2](1), mPoints[2](2);

        switch (mManhattanState){
            case NoLack:
                v1 *= mlam[0]; v2 *= mlam[1]; v3 *= mlam[2];
                cout << mCurrentFrame->Id() << " : " << "No Lack" << endl;
                break;
            case LackX:
                v2 *= mlam[1]; v3 *= mlam[2];
                v1 = v2.cross(v3);
                cout << mCurrentFrame->Id() << " : " << "LackX" << endl;
                break;
            case LackY:
                v1 *= mlam[0]; v3 *= mlam[2];
                v2 = v3.cross(v1);
                cout << mCurrentFrame->Id() << " : " << "LackY" << endl;
                break;
            case LackZ:
                v1 *= mlam[0]; v2 *= mlam[1];
                v3 = v1.cross(v2);
                cout << mCurrentFrame->Id() << " : " << "LackZ" << endl;
                break;
        }

        Eigen::Vector3f lastR;
        lastR = mLastR.col(0);
        if(v1.dot(lastR) < 0)
            v1 = -v1;
        lastR = mLastR.col(1);
        if(v2.dot(lastR) < 0)
            v2 = -v2;
        lastR = mLastR.col(2);
        if(v3.dot(lastR) < 0)
            v3 = -v3;

        mR << v1, v2, v3;
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(mR, Eigen::ComputeFullV | Eigen::ComputeFullU);
        const Eigen::Matrix3f& left_singular_vectors = svd.matrixU();
        const Eigen::Matrix3f& right_singular_vectors = svd.matrixV();
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
            float s = sqrt(p(0)*p(0) + p(1)*p(1));
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
            for (int i = 0; i < mCurrentFrame->RiemannPointCloud()[id]->points.size(); ++i) {
                PointT n = mCurrentFrame->RiemannPointCloud()[id]->points[i];
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
        for (int i = 0; i < mCurrentFrame->RiemannPointCloud()[id]->points.size(); ++i) {
            PointT n = mCurrentFrame->RiemannPointCloud()[id]->points[i];
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
