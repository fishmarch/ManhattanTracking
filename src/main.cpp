#include "Initializer.h"
#include "Tracking.h"
#include "base.h"
#include "System.h"

#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <ctime>
using namespace std;
using namespace MANHATTAN_TRACKING;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD);

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "usage: manhattanTracking #CONFIG.yml";
        return 0;
    }
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    string strAssociationFilename;
    string sequenceFolder;
    fs["associations_file"] >> strAssociationFilename;
    fs["sequence_folder"] >> sequenceFolder;

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    LoadImages(strAssociationFilename,vstrImageFilenamesRGB,vstrImageFilenamesD);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    //viewer.addCoordinateSystem(1.0);

    int nImages = vstrImageFilenamesRGB.size();
    cv::Mat rgb, depth;
    System SLAM(argv[1]);

    PointT origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    PointT p;
    PointT q;
    Eigen::Vector3f Rline;
    PrintString("Start Tracking");
    {
        rgb = cv::imread(sequenceFolder + "/" + vstrImageFilenamesRGB[0]);
        depth = cv::imread(sequenceFolder + "/" + vstrImageFilenamesD[0], -1);
        clock_t time1 = clock();
        SLAM.TrackFrame(depth, rgb);
        cout<< "Time of  Initialization : " << 1000*(clock() - time1)/(double)CLOCKS_PER_SEC << "ms" << endl;

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcolor(
                SLAM.Tracker()->CurrentFrame()->Cloud());
        viewer.addPointCloud(SLAM.Tracker()->CurrentFrame()->Cloud(), rgbcolor, "frame", v1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
                SLAM.Tracker()->CurrentFrame()->NormalPoints(), 0, 255, 255);
        viewer.addPointCloud(SLAM.Tracker()->CurrentFrame()->NormalPoints(), single_color, "normal", v2);

        Rline = SLAM.Tracker()->R().col(0);
        p.x = Rline(0); q.x = -p.x;
        p.y = Rline(1); q.y = -p.y;
        p.z = Rline(2); q.z = -p.z;
        viewer.addLine(q, p, 255, 0, 0, "x1", v2);
        Rline = SLAM.Tracker()->R().col(1);
        p.x = Rline(0); q.x = -p.x;
        p.y = Rline(1); q.y = -p.y;
        p.z = Rline(2); q.z = -p.z;
        viewer.addLine(q, p, 0, 255, 0, "y1", v2);
        Rline = SLAM.Tracker()->R().col(2);
        p.x = Rline(0); q.x = -p.x;
        p.y = Rline(1); q.y = -p.y;
        p.z = Rline(2); q.z = -p.z;
        viewer.addLine(q, p, 0, 0, 255, "z1", v2);

        rgb.release();
        depth.release();
    }

    for(int ni=1; ni<nImages; ni++) {
        rgb = cv::imread(sequenceFolder + "/" + vstrImageFilenamesRGB[ni]);
        depth = cv::imread(sequenceFolder + "/" + vstrImageFilenamesD[ni], -1);
        if(SLAM.TrackFrame(depth, rgb)) {

            viewer.removeAllPointClouds(v1);
            viewer.removeAllPointClouds(v2);
            viewer.removeAllShapes(v2);

            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcolor(
                    SLAM.Tracker()->CurrentFrame()->Cloud());
            viewer.addPointCloud(SLAM.Tracker()->CurrentFrame()->Cloud(), rgbcolor, "frame", v1);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
                    SLAM.Tracker()->CurrentFrame()->NormalPoints(), 0, 255, 255);
            viewer.addPointCloud(SLAM.Tracker()->CurrentFrame()->NormalPoints(), single_color, "normal", v2);

            Rline = SLAM.Tracker()->R().col(0);
            p.x = Rline(0); q.x = -p.x;
            p.y = Rline(1); q.y = -p.y;
            p.z = Rline(2); q.z = -p.z;
            viewer.addLine(q, p, 255, 0, 0, "x1", v2);
            Rline = SLAM.Tracker()->R().col(1);
            p.x = Rline(0); q.x = -p.x;
            p.y = Rline(1); q.y = -p.y;
            p.z = Rline(2); q.z = -p.z;
            viewer.addLine(q, p, 0, 255, 0, "y1", v2);
            Rline = SLAM.Tracker()->R().col(2);
            p.x = Rline(0); q.x = -p.x;
            p.y = Rline(1); q.y = -p.y;
            p.z = Rline(2); q.z = -p.z;
            viewer.addLine(q, p, 0, 0, 255, "z1", v2);
        }
        rgb.release();
        depth.release();
        viewer.spinOnce();
    }


    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}