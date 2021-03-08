#pragma once

#ifndef LEGOSLAM_FEATURE_H
#define LEGOSLAM_FEATURE_H

#include <memory>
//#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <utility>
#include "legoslam/common_include.h"
#include "legoslam/frame.h"

namespace legoslam {

    class Frame;
    class MapPoint;

    /*
     * 2D features
     * will correspond with a 3D map point after trangulation
     */
    class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;

        // the frame holding this feature
        std::weak_ptr<Frame> frame_;
        // the map point holding this feature
        std::weak_ptr<MapPoint> map_point_;
        // feature 2D position
        cv::KeyPoint position_;

        // is outlier or not
        bool is_outlier_ = false;
        // on the left or right img -> default: on the left image rather than the right image
        bool is_on_left_image_ = true;
        bool is_on_right_image_ = false;

    public:
        // constructor
        Feature() = default;
        //
        Feature(const std::shared_ptr<Frame>& frame, cv::KeyPoint kp) : frame_(frame), position_(std::move(kp)) {}
    };
}
#endif  // LEGOSLAM_FEATURE_H
