//
// Created by fishmarch on 19-4-23.
//

#ifndef MANHATTANTRACKING_SYSTEM_H
#define MANHATTANTRACKING_SYSTEM_H

#include "Tracking.h"

namespace MANHATTAN_TRACKING{
    class System{
    public:
        System(char* ConfigFile);
        void TrackFrame(const cv::Mat& depth, const cv::Mat& rgb);
    private:
        Tracking* mTracker;
        char* mConfigFile;
    };
}




#endif //MANHATTANTRACKING_SYSTEM_H
