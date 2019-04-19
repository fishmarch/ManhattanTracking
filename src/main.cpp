#include "Initializer.h"
#include "Tracking.h"
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace MANHATTAN_TRACKING;
int main(int argc, char **argv){
    if (argc != 2) {
        cout << "usage: manhattanTracking #CONFIG.yml";
        return 0;
    }
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    string gaussian_path, gaussian_path2;
    int trytim;
    float window;
    bool UseGaussianCore;
    fs["gaussian_path"] >> gaussian_path;
    fs["gaussian_path2"] >> gaussian_path2;
    fs["initilize_time"] >> trytim;
    fs["window_size"] >> window;
    fs["UseGaussianCore"] >> UseGaussianCore;

    PointCloud::Ptr inputcloud(new PointCloud);
    pcl::io::loadPCDFile(gaussian_path, *inputcloud);

    PointCloud::Ptr inputcloud2(new PointCloud);
    pcl::io::loadPCDFile(gaussian_path2, *inputcloud2);

    Initializer init(inputcloud, trytim, window, UseGaussianCore);
    if(init.Initialize()){
        cout << "Initialize successfully !" << endl;
        cout << init.R12() << endl;
    }

    Tracking track(inputcloud2, init.R12(), window, UseGaussianCore);
    if(track.Track()){
        cout << "Tracking successfully !" << endl;
        cout << track.R12() << endl;
    }
}
