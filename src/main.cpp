#include "Initializer.h"

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv){
    if (argc != 2) {
        cout << "usage: manhattanTracking #CONFIG.yml";
        return 0;
    }
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    string gaussian_path;    
    int trytim;
    float window;
    bool UseGaussianCore;
    fs["gaussian_path"] >> gaussian_path;
    fs["initilize_time"] >> trytim;
    fs["window_size"] >> window;
    fs["UseGaussianCore"] >> UseGaussianCore;

    PointCloud::Ptr inputcloud(new PointCloud);
    pcl::io::loadPCDFile(gaussian_path, *inputcloud);

    MANHATTAN_TRACKING::Initializer SLAM(inputcloud, trytim, window, UseGaussianCore);
    if(SLAM.Initialize()){
        cout << "Initialize successfully !" << endl;
        cout << SLAM.R12().transpose() << endl;
    }
}
