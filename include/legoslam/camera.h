#pragma once

#ifndef LEGOSLAM_CAMERA_H
#define LEGOSLAM_CAMERA_H

#include "legoslam/common_include.h"

namespace legoslam {

    /*
     * pinhole stereo camera model
     */
    class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Camera> Ptr;

        // camera intrinsics
        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;
        // extrinsics: from stereo camera to single camera
        SE3 pose_;
        // inverse of extrinsics
        SE3 pose_inv_;

    public:
        Camera();

        Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
            pose_inv_ = pose_.inverse();
        }

        SE3 pose() const { return pose_; }

        // intrinsics matrix
        Mat33 K() const {
            Mat33 k;
            k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;

            return k;
        }

        // coordinates transform
        Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

        Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

        Vec2 camera2pixel(const Vec3 &p_c);

        Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

        Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

        Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
    };
}
#endif  // LEGOSLAM_CAMERA_H
