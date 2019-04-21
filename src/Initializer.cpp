#include "Initializer.h"

namespace MANHATTAN_TRACKING{

    Initializer::Initializer(PointCloud::Ptr InitializationPointCloud, int tim, float window,
                             bool UseGaussianCore,
                             pcl::visualization::PCLVisualizer& viewer, int& port)
    : mInitializationPointCloud(InitializationPointCloud), mPort(port),
    mRiemannPointCloud(new PointCloud), mMeanShiftPoints(new PointCloud),
    mTime(tim), mWindow(window), mUseGaussianCore(UseGaussianCore), mViewer(viewer)
    {
        RiemannMapping();
    }

    bool Initializer::Initialize(){
        ClearData();
        int tim = mTime;
        default_random_engine e(time(0));
        uniform_real_distribution<float> random(-1,1);
        
        //meanshift from random points, repeat tim times
        while (tim--){
            float x_mean = random(e);
            float y_mean = random(e);
            if(mUseGaussianCore)
                MeanShift(x_mean, y_mean, DisGaussianCore);
            else
                MeanShift(x_mean, y_mean, DisUniformCore);

            PointT p;
            p.x = x_mean;
            p.y = y_mean;
            p.z = 1.1;
            mMeanShiftPoints->points.push_back(p);    
        }

        //sort the meanshitf results by point.x
        sort(mMeanShiftPoints->points.begin(),mMeanShiftPoints->points.end(),[](PointT& a,PointT& b){return a.x < b.x; });

        //try to find three centers 
        float last = -2;
        float sum_x = 0;
        float sum_y = 0;
        float sum_num = 0;
        for (int i = 0; i < mMeanShiftPoints->points.size(); ++i) {
            PointT& p = mMeanShiftPoints->points[i];
            //cout << "final point: " << p.x << " , " << p.y << endl;
            if(p.x-last > 0.01){
                //cout << sum_num << endl;
                if(sum_num > mTime/10){
                    PointT n;
                    n.x = sum_x/sum_num;
                    n.y = sum_y/sum_num;
                    mInitPoints.push_back(n);
                }
                last = p.x;
                sum_x = p.x;
                sum_y = p.y;
                sum_num = 1;
            }else{
                sum_num++;
                sum_x += p.x;
                sum_y += p.y;
                last = p.x;
            }
        }    
        if(sum_num > mTime/10){
            PointT n;
            n.x = sum_x/sum_num;
            n.y = sum_y/sum_num;
            mInitPoints.push_back(n);
        }

        if(mInitPoints.size()!=3){
            cout << "init points size : " << mInitPoints.size() << endl;
            return false;
        }

        for(auto& p:mInitPoints){
            float p1 = KernelDensityEstimate(p.x, p.y, DisUniformCore);
            float p2 = KernelDensityEstimate(p.x, p.y, DisGaussianCore);
            float lam = p2/p1;
            //cout << " lam : " << lam << endl;
            mlam.push_back(lam);
        }

        
        RiemannUnmapping();
        
        Eigen::Vector3f v1; v1 << mInitPoints[0].x, mInitPoints[0].y, mInitPoints[0].z;
        Eigen::Vector3f v2; v2 << mInitPoints[1].x, mInitPoints[1].y, mInitPoints[1].z;
        Eigen::Vector3f v3; v3 << mInitPoints[2].x, mInitPoints[2].y, mInitPoints[2].z;

        v1 *= mlam[0]; v2 *= mlam[1]; v3 *= mlam[2];

        Eigen::Vector3f v12 = v1.cross(v2);
        if(v12.dot(v3)<0){
            swap(v1,v2);
        }
        mR << v1, v2, v3;

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(mR, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::Matrix3f left_singular_vectors = svd.matrixU();
        Eigen::Matrix3f right_singular_vectors = svd.matrixV();

        mR = left_singular_vectors*right_singular_vectors.transpose();

        return true;
    }
    
    float Initializer::KernelDensityEstimate(float x_mean, float y_mean, float (*dis)(float, float, float, float, float)){
        float weights = 0;
        for (int i = 0; i < mRiemannPointCloud->points.size(); ++i) {
                PointT n = mRiemannPointCloud->points[i];
                if (isnan(n.x))
                    continue;
                float weight = dis(n.x, n.y, x_mean, y_mean, mWindow);
                weights += weight;
        }
        return weights;
    }


    void Initializer::RiemannMapping(){
        for (int i = 0; i < mInitializationPointCloud->points.size(); ++i) {
            PointT p;
            PointT n = mInitializationPointCloud->points[i];
            if(isnan(n.x))
                continue;
            float lam = sqrt(n.x * n.x + n.y * n.y);
            float theta = asin(lam);
            p.x = theta * n.x / lam;
            p.y = theta * n.y / lam;
            if(n.z < 0){
                p.x = -p.x;
                p.y = -p.y;
            }
            p.z = 1;

            mRiemannPointCloud->points.push_back(p);
        }
        mRiemannPointCloud->is_dense = false;
    }

    void Initializer::RiemannUnmapping(){
        for(auto& p : mInitPoints){
            float s = p.x*p.x + p.y*p.y;
            s = sqrt(s);
            float th = tan(s);
            p.x = th*p.x/s;
            p.y = th*p.y/s;
            p.z = 1;
            float norm = p.x*p.x + p.y*p.y + p.z*p.z;
            norm = sqrt(norm);
            p.x = -p.x/norm;
            p.y = -p.y/norm;
            p.z = -p.z/norm;
        }
    }

    float Initializer::MeanShift(float& x_mean, float& y_mean, float (*dis)(float, float, float, float, float)){
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
                for (int i = 0; i < mRiemannPointCloud->points.size(); ++i) {
                    PointT n = mRiemannPointCloud->points[i];
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

    void Initializer::ClearData() {
        mlam.clear();
        mInitPoints.clear();
        mMeanShiftPoints->points.clear();
    }

    float Initializer::DisUniformCore(float x, float y, float x_mean, float y_mean, float window){
        float x_dis = x - x_mean;
        float y_dis = y - y_mean;
        if (pow(x_dis, 2) + pow(y_dis, 2) > window*window)
            return 0;
        else
            return 1;
    }

    float Initializer::DisGaussianCore(float x, float y, float x_mean, float y_mean, float window){
        float x_dis = x - x_mean;
        float y_dis = y - y_mean;
        float dis = pow(x_dis, 2) + pow(y_dis, 2);
        float sigma = window / 3;

        float res = exp(-dis / (2 * sigma * sigma));
        //res = res / (sqrt(2 * pi) * sigma);
        return res;
    }
}