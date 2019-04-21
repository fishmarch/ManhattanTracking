#include "Initializer.h"
#include "Tracking.h"
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace MANHATTAN_TRACKING;

int main(int argc, char **argv) {
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

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    viewer.addCoordinateSystem(1.0);
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    PointCloud::Ptr inputcloud(new PointCloud);
    pcl::io::loadPCDFile(gaussian_path, *inputcloud);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(inputcloud, 0, 255, 255);
    viewer.addPointCloud(inputcloud, single_color, "input1", v1);

    PointCloud::Ptr inputcloud2(new PointCloud);
    pcl::io::loadPCDFile(gaussian_path2, *inputcloud2);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(inputcloud2, 0, 255, 255);
    viewer.addPointCloud(inputcloud2, single_color2, "input2", v2);

    Initializer init(inputcloud, trytim, window, UseGaussianCore, viewer, v1);
    PointT origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    int init_tim = 10;
    while (init_tim--) {
        if (init.Initialize()) {
            cout << "Initialize successfully !" << endl;
            cout << init.R12() << endl;
            Eigen::Vector3f R1 = init.R12().col(0);
            Eigen::Vector3f R2 = init.R12().col(1);
            Eigen::Vector3f R3 = init.R12().col(2);
            PointT p;
            p.x = R1(0);
            p.y = R1(1);
            p.z = R1(2);
            viewer.addLine(origin, p, 255, 0, 0, "x1", v1);
            p.x = R2(0);
            p.y = R2(1);
            p.z = R2(2);
            viewer.addLine(origin, p, 0, 255, 0, "y1", v1);
            p.x = R3(0);
            p.y = R3(1);
            p.z = R3(2);
            viewer.addLine(origin, p, 0, 0, 255, "z1", v1);
            break;
        }
    }
    if (init_tim == 0) {
        cout << "initialize failed" << endl;
        return 0;
    }

    Tracking track(inputcloud2, init.R12(), window, UseGaussianCore, viewer, v2);
    if (track.Track()) {
        cout << "Tracking successfully !" << endl;
        cout << track.R12() << endl;
        Eigen::Vector3f R1 = track.R12().col(0);
        Eigen::Vector3f R2 = track.R12().col(1);
        Eigen::Vector3f R3 = track.R12().col(2);
        PointT p;
        p.x = R1(0);
        p.y = R1(1);
        p.z = R1(2);
        viewer.addLine(origin, p, 255, 0, 0, "x2", v2);
        p.x = R2(0);
        p.y = R2(1);
        p.z = R2(2);
        viewer.addLine(origin, p, 0, 255, 0, "y2", v2);
        p.x = R3(0);
        p.y = R3(1);
        p.z = R3(2);
        viewer.addLine(origin, p, 0, 0, 255, "z2", v2);
    }

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

}
