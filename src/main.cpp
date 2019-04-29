#include "Initializer.h"
#include "Tracking.h"
#include "base.h"
#include "System.h"

#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <ctime>
#include <iomanip>
#include <fstream>
using namespace std;
using namespace MANHATTAN_TRACKING;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vstrImageFilenamesTime);

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "usage: manhattanTracking #CONFIG.yml";
        return 0;
    }
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    string strAssociationFilename;
    string sequenceFolder;
    string resultPath;

    fs["associations_file"] >> strAssociationFilename;
    fs["sequence_folder"] >> sequenceFolder;
    fs["result_path"] >> resultPath;

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vstrImageFilenamesTime;

    LoadImages(strAssociationFilename,vstrImageFilenamesRGB,vstrImageFilenamesD,vstrImageFilenamesTime);

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

    ofstream resultFile;
    resultFile.open(resultPath);

    Eigen::Quaternionf rotation;
    Eigen::Quaternionf world(0.2239, -0.4871, 0.7673, -0.3519);


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

        rotation = SLAM.Tracker()->R();
        resultFile << fixed << vstrImageFilenamesTime[0] << " " << 0 << " " << 0 << " " << 0 << " "
        //<< rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << endl;
        << world.x() << " " << world.y() << " " << world.z() << " " << world.w() << endl;

        world = world * rotation;
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
            p.x = 1.5*Rline(0); q.x = -p.x;
            p.y = 1.5*Rline(1); q.y = -p.y;
            p.z = 1.5*Rline(2); q.z = -p.z;
            viewer.addLine(origin, p, 255, 0, 0, "x1", v2);
            Rline = SLAM.Tracker()->R().col(1);
            p.x = 1.5*Rline(0); q.x = -p.x;
            p.y = 1.5*Rline(1); q.y = -p.y;
            p.z = 1.5*Rline(2); q.z = -p.z;
            viewer.addLine(origin, p, 0, 255, 0, "y1", v2);
            Rline = SLAM.Tracker()->R().col(2);
            p.x = 1.5*Rline(0); q.x = -p.x;
            p.y = 1.5*Rline(1); q.y = -p.y;
            p.z = 1.5*Rline(2); q.z = -p.z;
            viewer.addLine(origin, p, 0, 0, 255, "z1", v2);
            rotation = SLAM.Tracker()->R();
            rotation = world * rotation.conjugate();

            resultFile << fixed << vstrImageFilenamesTime[ni] << " " << 0 << " " << 0 << " " << 0 << " "
                       << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << endl;
        }
        rgb.release();
        depth.release();
        viewer.spinOnce();
    }

    resultFile.close();

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vstrImageFilenamesTime)
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
            vstrImageFilenamesTime.push_back(t);
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}