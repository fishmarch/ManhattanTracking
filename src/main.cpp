#include "Initializer.h"
#include "Tracking.h"
#include "base.h"
#include "System.h"

#include <pcl/visualization/cloud_viewer.h>
#include <vector>
using namespace std;
using namespace MANHATTAN_TRACKING;
typedef pcl::PointXYZRGB PointT;
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

    int nImages = vstrImageFilenamesRGB.size();
    cv::Mat rgb, depth;
    System SLAM(argv[1]);

    PrintString("Start Tracking");
    for(int ni=0; ni<nImages; ni++) {
        rgb = cv::imread(sequenceFolder + "/" + vstrImageFilenamesRGB[ni]);
        depth = cv::imread(sequenceFolder + "/" + vstrImageFilenamesD[ni], -1);
        SLAM.TrackFrame(depth, rgb);
        rgb.release();
        depth.release();
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