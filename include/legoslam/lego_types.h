#ifndef LEGOSLAM_G2O_TYPES_H
#define LEGOSLAM_G2O_TYPES_H

#include "legoslam/common_include.h"

// LEGO lib
//#include <lego/base/base_vertex.h>
//#include <lego/base/base_edge.h>
//#include <lego/base/cost_function.h>
//#include <lego/base/problem.h>

// lego base
#include "../../src/lego/base/base_vertex.h"
#include "../../src/lego/base/base_edge.h"
#include "../../src/lego/base/cost_function.h"
#include "../../src/lego/base/problem.h"

namespace legoslam {

    /*
     * vertex and edge in lego Bundle Adjustment
     */

    // VertexPose: pose vertex
    class VertexPose : public lego::BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPose() : lego::BaseVertex(4, 4, 6) {
            // 7D
//            estimate_ = Vec7::Zero();

            // 6D
//            estimate_ = Vec6::Zero();

            // SE3
            estimate_ = Mat44::Zero();
        }

        void setEstimate(const SE3 &T) {
            // 7D
//            const auto &t = T.translation();
//            const auto R = T.rotationMatrix();
//            auto q = Eigen::Quaterniond(R);
//            q.normalize();

//            estimate_.head<3>() = t.head<3>();
//            estimate_[3] = q.x();
//            estimate_[4] = q.y();
//            estimate_[5] = q.z();
//            estimate_[6] = q.w();

            // 6D
//            estimate_ = T.log();

            // SE3
            estimate_ = T.matrix();
        }

        // optimization is perform on manifold, so update is 6 DoF, left multiplication
        void add(const VecX &update) override {
            Vec6 update_eigen;
            if (std::isnan(update[0]) || std::isnan(update[1]) || std::isnan(update[2]) ||
                std::isnan(update[3]) || std::isnan(update[4]) || std::isnan(update[5]) ||
                std::isinf(update[0]) || std::isinf(update[1]) || std::isinf(update[2]) ||
                std::isinf(update[3]) || std::isinf(update[4]) || std::isinf(update[5]))
                update_eigen.setZero();
            else
                update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];

            // 7D
//            // translation
//            estimate_.head<3>() += update_eigen.head<3>();
//            // rotation
//            Eigen::Quaterniond q(estimate_[6], estimate_[3], estimate_[4], estimate_[5]);
//            // right multiplication on so3
//            //q = q * SO3::exp(Vec3(update_eigen[3], update_eigen[4], update_eigen[5])).unit_quaternion();
//            // left multiplication on so3
//            q = SO3::exp(Vec3(update_eigen[3], update_eigen[4], update_eigen[5])).unit_quaternion() * q;
//            q.normalize();  // normalization of quaternion
//            estimate_[3] = q.x();
//            estimate_[4] = q.y();
//            estimate_[5] = q.z();
//            estimate_[6] = q.w();

            // 6D
//            estimate_ = (SE3::exp(update_eigen) * SE3::exp(estimate_)).log();

            // SE3
            estimate_ = (SE3::exp(update_eigen) * SE3(estimate_)).matrix();
        }

        std::string getInfo() const override { return "VertexPose"; }
    };

    // VertexXYZ: landmark point vertex
    class VertexXYZ : public lego::BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexXYZ() : lego::BaseVertex(3) {
            estimate_ = Vec3::Zero();
        }

        void add(const VecX &update) override {
            if (!std::isnan(update[0]) && !std::isnan(update[1]) && !std::isnan(update[2]) &&
                !std::isinf(update[0]) && !std::isinf(update[1]) && !std::isinf(update[2])) {
                estimate_(0, 0) += update[0];
                estimate_(1, 0) += update[1];
                estimate_(2, 0) += update[2];
            }
        }

        std::string getInfo() const override { return "VertexXYZ"; }
    };

    // EdgeProjectionPoseOnly: unary edge just estimate pose only
    class EdgeProjectionPoseOnly : public lego::BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K)
            : lego::BaseEdge(2, 1, 2, 1,
                             std::vector<std::string>{"VertexPose"}), _pos3d(pos), _K(K) {
            residual_ = Vec2::Zero();
            measurement_ = Vec2::Zero();
        }

        void computeResidual() override {
            // 7D
//            const Vec7 v_pose_est = vertexes_[0]->getEstimate();
//            SE3 T(Eigen::Quaterniond(v_pose_est[6], v_pose_est[3], v_pose_est[4], v_pose_est[5]),
//                           v_pose_est.head<3>());

            // 6D
//            const Vec6 v_pose_est = vertexes_[0]->getEstimate();
//            SE3 T = SE3::exp(v_pose_est);

            // SE3
            SE3 T = SE3(vertexes_[0]->getEstimate());

            Vec3 pos_pixel = _K * (T * _pos3d);
            pos_pixel /= (pos_pixel[2] + 1e-18);
            residual_ = measurement_ - pos_pixel.head<2>();
        }

        void computeJacobians() override {
            // 7D
//            const Vec7 v_pose_est = vertexes_[0]->getEstimate();
//            SE3 T(Eigen::Quaterniond(v_pose_est[6], v_pose_est[3], v_pose_est[4], v_pose_est[5]),
//                  v_pose_est.head<3>());

            // 6D
//            const Vec6 v_pose_est = vertexes_[0]->getEstimate();
//            SE3 T = SE3::exp(v_pose_est);

            // SE3
            SE3 T = SE3(vertexes_[0]->getEstimate());

            Vec3 pos_cam = T * _pos3d;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            // X, Y, Z: the 3D point coordinate in the camera coordinate after transformation
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;

            // jacobians
            jacobians_[0] = Eigen::Matrix<double, 2, 6>::Zero();
            Eigen::Matrix<double, 2, 6> j_i;
            j_i <<
                -fx*Zinv, 0, fx*X*Zinv2, fx*X*Y*Zinv2, -fx-fx*X*X*Zinv2, fx*Y*Zinv,
                    0, -fy*Zinv, fy*Y*Zinv2, fy+fy*Y*Y*Zinv2, -fy*X*Y*Zinv2, -fy*X*Zinv;

            jacobians_[0].block<2, 6>(0, 0) = j_i;
        }

        std::string getInfo() const override { return "EdgeProjectionPoseOnly"; }

    private:
        Vec3 _pos3d;
        Mat33 _K;
    };

    // EdgeProjection: edge between pose and landmark point
    class EdgeProjection : public lego::BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Mat33 &K, const SE3 &cam_ext) :
            lego::BaseEdge(2, 2, 2, 1,
                           std::vector<std::string>{"VertexPose", "VertexXYZ"}), _K(K) {
            residual_ = Vec2::Zero();
            measurement_ = Vec2::Zero();
            _cam_ext = cam_ext;
        }

        void computeResidual() override {
            // 7D
//            const Vec7 v0_pose_est = vertexes_[0]->getEstimate();
//            SE3 T(Eigen::Quaterniond(v0_pose_est[6], v0_pose_est[3], v0_pose_est[4], v0_pose_est[5]),
//                           v0_pose_est.head<3>());

            // 6D
//            const Vec6 v0_pose_est = vertexes_[0]->getEstimate();
//            SE3 T = SE3::exp(v0_pose_est);

            // SE3
            SE3 T = SE3(vertexes_[0]->getEstimate());

            Vec3 pos_pixel = _K * (_cam_ext * (T * Vec3(vertexes_[1]->getEstimate())));
            pos_pixel /= (pos_pixel[2] + 1e-18);
            residual_ = measurement_ - pos_pixel.head<2>();
        }

        void computeJacobians() override {
            // 7D
//            const Vec7 v0_pose_est = vertexes_[0]->getEstimate();
//            SE3 T(Eigen::Quaterniond(v0_pose_est[6], v0_pose_est[3], v0_pose_est[4], v0_pose_est[5]),
//                           v0_pose_est.head<3>());

            // 6D
//            const Vec6 v0_pose_est = vertexes_[0]->getEstimate();
//            SE3 T = SE3::exp(v0_pose_est);

            // SE3
            SE3 T = SE3(vertexes_[0]->getEstimate());

            const Vec3 &pw = Vec3(vertexes_[1]->getEstimate());
            Vec3 pos_cam = _cam_ext * T * pw;

            double fx = _K(0, 0);
            double fy = _K(1, 1);
            // X, Y, Z: the 3D point coordinate in the camera coordinate after transformation
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;

            // jacobians
            jacobians_[0] = Eigen::Matrix<double, 2, 6>::Zero();
            Eigen::Matrix<double, 2, 6> j_i;
            j_i <<
                -fx*Zinv, 0, fx*X*Zinv2, fx*X*Y*Zinv2, -fx-fx*X*X*Zinv2, fx*Y*Zinv,
                0, -fy*Zinv, fy*Y*Zinv2, fy+fy*Y*Y*Zinv2, -fy*X*Y*Zinv2, -fy*X*Zinv;
            jacobians_[0].block<2, 6>(0, 0) = j_i;

            Eigen::Matrix<double, 2, 3> j_j_part = j_i.block<2, 3>(0, 0);
            jacobians_[1] = Eigen::Matrix<double, 2, 3>::Zero();
            jacobians_[1].block<2, 3>(0, 0) =  j_j_part * _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        std::string getInfo() const override { return "EdgeProjection"; }

    private:
        Mat33 _K;
        SE3 _cam_ext;
    };
}
#endif  // LEGOSLAM_G2O_TYPES_H
