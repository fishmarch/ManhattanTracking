//
// Created by fishmarch on 19-4-23.
//

#include "System.h"

namespace MANHATTAN_TRACKING{
    System::System(char *ConfigFile) : mConfigFile(ConfigFile){
        mTracker = new Tracking(mConfigFile);

        cv::FileStorage fs(mConfigFile, cv::FileStorage::READ);
        float camera_factor = 5000;
        float camera_cx = 320.1;
        float camera_cy = 247.6;
        float camera_fx = 535.4;
        float camera_fy = 539.2;
        fs["camera_factor"] >> camera_factor;
        fs["camera_cx"] >> camera_cx;
        fs["camera_cy"] >> camera_cy;
        fs["camera_fx"] >> camera_fx;
        fs["camera_fy"] >> camera_fy;
        Frame::SetCameraParameter(camera_fx, camera_fy, camera_cx, camera_cy, camera_factor);
    }

    bool System::TrackFrame(const cv::Mat &depth, const cv::Mat &rgb) {
        return mTracker->GrabFrame(depth, rgb);
    }
}