//
// Created by deeprealtech on 2022/3/18.
//

#ifndef ORB_SLAM2_DENSERECONS_H
#define ORB_SLAM2_DENSERECONS_H

#include "System.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "open3d/pipelines/integration/ScalableTSDFVolume.h"
#include "open3d/pipelines/integration/TSDFVolume.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/Image.h"
#include "open3d/geometry/RGBDImage.h"
#include "open3d/camera/PinholeCameraIntrinsic.h"
#include <condition_variable>
#include <mutex>
using namespace open3d;
namespace ORB_SLAM2
{
    class DenseRecons
    {
    public:
        DenseRecons();
        void insertKeyFrame( KeyFrame* kf);
        void shutdown();
        void viewer();
        std::shared_ptr<geometry::PointCloud> GetPointCloud();
    protected:
        std::shared_ptr<geometry::PointCloud> globalMap;
        shared_ptr<thread>  viewerThread;

        bool    shutDownFlag    =false;
        mutex   shutDownMutex;

        condition_variable  keyFrameUpdated;
        mutex               keyFrameUpdateMutex;

        // data to generate point clouds
        vector<KeyFrame*>       keyframes;
        mutex                   keyframeMutex;
        uint16_t                lastKeyframeSize =0;
    };
}


#endif //ORB_SLAM2_DENSERECONS_H
