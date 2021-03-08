#pragma once

#ifndef LEGOSLAM_FRONTEND_H
#define LEGOSLAM_FRONTEND_H

#include <opencv2/features2d/features2d.hpp>

#include "legoslam/common_include.h"
#include "legoslam/frame.h"
#include "legoslam/map.h"

namespace legoslam {

    class Backend;
    class Viewer;

    enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

    /*
     * Frontend:
     * estimate pose of the current frame, add keyframe into map when appropriate,
     * and trigger the backend optimization
     */
    class Frontend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        // add a frame and compute pose
        bool AddFrame(Frame::Ptr frame);
        // set functions
        void SetMap(Map::Ptr map) { map_ = map; }
        // set backend
        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }
        // set viewer
        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
        // set camera
        void SetCameras(Camera::Ptr left, Camera::Ptr right) {
            camera_left_ = left;
            camera_right_ = right;
        }
        // get front-end status
        FrontendStatus GetStatus() const { return status_; }

    private:
        // track in normal mode
        bool Track();
        // reset when lost
        bool Reset();
        // track with last frame
        int TrackLastFrame();
        // track last frame with OpenCV LK optical flow
        int TrackLastFrameLKOpticalFlowOpenCV();
        // track last frame with four layers LK optical flow with Gauss-Newton method
        int TrackLastFrameLKOpticalFlow4LayerSelf();
        // find features in right with OpenCV optical flow
        int FindFeaturesInRightLKOpticalFlowOpenCV();
        // find features in right with four layers LK optical flow with Gauss-Newton method
        int FindFeaturesInRightLKOpticalFlow4LayerSelf();
        // estimate the pose of current frame
        int EstimateCurrentPose();
        // set current frame as a keyframe and insert it into backend
        bool InsertKeyframe();
        // try init the frontend with stereo images saved in current_frame_
        bool StereoInit();
        // detect features in left image in current_frame_
        int DetectFeatures();
        // find corresponding features in right image of current_image_
        int FindFeaturesInRight();
        // build initial map with single image
        bool BuildInitMap();
        // triangulate 2D points in current frame
        int TriangulateNewPoints();
        // set features in keyframe as new observations of the map points
        void SetObservationForKeyFrame();

        // data member
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        // relative motion between current frame and last frame,
        // for estimating the initial value of pose of current frame
        SE3 relative_motion_ = SE3(Eigen::Matrix4d::Identity());

        // inliers, used for testing new keyframes
        int tracking_inliers_ = 0;

        // parameters
        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 30;
        int num_features_tracking_bad_ = 5;
        int num_features_needed_for_keyframe_ = 80;
        // limits of point depth from triangulation
        double stereo_depth_superior_limit_ = 1000;
        double stereo_depth_inferior_limit_ = 0;

        // utilities: GFTT feature detector in OpenCV
        cv::Ptr<cv::GFTTDetector> gftt_;
    };
}
#endif  // LEGOSLAM_FRONTEND_H
