#pragma once

#ifndef LEGOSLAM_FRAME_H
#define LEGOSLAM_FRAME_H

#include "legoslam/camera.h"
#include "legoslam/common_include.h"

namespace legoslam {

    class MapPoint;
    class Feature;

    /*
     * Frame: a general frame with a unique id, a keyframe with a keyframe id
     */
    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        // id
        unsigned long id_ = 0;
        // keyframe id
        unsigned long keyframe_id_ = 0;
        // id keyframe or not
        bool is_keyframe_ = false;
        bool is_active_keyframe_ = false;
        // tine stamp
        double time_stamp_{};
        // pose: T_c_w
        SE3 pose_;
        // pose data lock
        std::mutex pose_mutex_;
        // stereo images
        cv::Mat left_img_, right_img_;

        // features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // corresponding features in right image, set to nullptr if none
        std::vector<std::shared_ptr<Feature>> features_right_;

    public:
        Frame() = default;

        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

        // set and get pose, thread safe
        SE3 Pose() {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }
        void SetPose(const SE3 &pose) {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }
        // set keyframe and keyframe id
        void SetKeyFrame();
        // static factory function: set id
        static Frame::Ptr CreateFrame();
    };
}
#endif  // LEGOSLAM_FRAME_H
