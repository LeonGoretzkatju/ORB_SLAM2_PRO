//
// Created by deeprealtech on 2022/3/18.
//
#include "DenseRecons.h"
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"
#include <boost/make_shared.hpp>



namespace ORB_SLAM2
{

    DenseRecons::DenseRecons() {
        globalMap = std::shared_ptr<geometry::PointCloud>();
        viewerThread = make_shared<thread>( bind(&DenseRecons::viewer, this ) );
    }

    void DenseRecons::shutdown()
    {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }

    void DenseRecons::insertKeyFrame(KeyFrame* kf)
    {
//        cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
        unique_lock<mutex> lck(keyframeMutex);
        keyframes.push_back( kf );

        keyFrameUpdated.notify_one();
    }

    std::shared_ptr<geometry::PointCloud> DenseRecons::GetPointCloud() {
        unique_lock<mutex> lck( keyframeMutex );
        return globalMap;
    }

    int num_i = 0;

    void DenseRecons::viewer() {
        pangolin::CreateWindowAndBind("Point Cloud Viewer", 720, 540);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));
        while (true)
        {

            {
                unique_lock<mutex> lck_shutdown( shutDownMutex );
                if (shutDownFlag)
                {
                    break;
                }
            }
            {
                unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
                keyFrameUpdated.wait( lck_keyframeUpdated );
            }

            // keyframe is updated
            size_t N=0;
            {
                unique_lock<mutex> lck( keyframeMutex );
                N = keyframes.size();
            }

            double length = 4.0;
            int resolution = 316.0;
            double sdf_trunc_percentage = 0.01;
            open3d::pipelines::integration::ScalableTSDFVolume volume(
                    length / (double)resolution, length * sdf_trunc_percentage,
                    open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
            if (N >= 1)
            {
                for ( size_t i=lastKeyframeSize; i<N ; i++ )
                {
                    if(keyframes[i]->isBad())
                        continue;

                    open3d::geometry::Image depth, color;

                    color.FromCVMatRGB(keyframes[i]->mRGB);
                    depth.FromCVMatRGB(keyframes[i]->mDepth);


                    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
                    fx = keyframes[i]->fx; fy = keyframes[i]->fy;
                    cx = keyframes[i]->cx; cy = keyframes[i]->cy;

                    auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                            color, depth, 5000.0, 7.0, false);
                    open3d::camera::PinholeCameraIntrinsic intrinsic_
                            (depth.width_, depth.height_, fx, fy, cx, cy );
                    cv::Mat extrinsic = keyframes[i]->GetPose();// Inverse();

                    Eigen::Matrix4d poseMat;
                    poseMat << extrinsic.at<float>(0,0), extrinsic.at<float>(0,1), extrinsic.at<float>(0,2), extrinsic.at<float>(0,3),
                            extrinsic.at<float>(1,0), extrinsic.at<float>(1,1), extrinsic.at<float>(1,2), extrinsic.at<float>(1,3),
                            extrinsic.at<float>(2,0), extrinsic.at<float>(2,1), extrinsic.at<float>(2,2), extrinsic.at<float>(2,3),
                            extrinsic.at<float>(3,0), extrinsic.at<float>(3,1), extrinsic.at<float>(3,2), extrinsic.at<float>(3,3);
                    volume.Integrate(*rgbd, intrinsic_,   poseMat);
                }
                auto pcd = volume.ExtractPointCloud();
                if (num_i == 0)
                    globalMap = pcd;
                else
                    *globalMap += *pcd;
                num_i++;
                lastKeyframeSize = N;
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                d_cam.Activate(s_cam);
                glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

                glPointSize(2);
                glBegin(GL_POINTS);
                for (int i = 0; i < globalMap->points_.size(); ++i) {
                    Eigen::Vector3d color = globalMap->colors_[i];
                    glColor3f(color[0], color[1], color[2]);
                    Eigen::Vector3d point = globalMap->points_[i];
                    glVertex3f(point[0], point[1], point[2]);
                }
                glEnd();
                pangolin::FinishFrame();

            }

        }
    }

}
