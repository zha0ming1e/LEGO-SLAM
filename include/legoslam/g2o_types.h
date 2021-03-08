#ifndef LEGOSLAM_G2O_TYPES_H
#define LEGOSLAM_G2O_TYPES_H

#include "legoslam/common_include.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace legoslam {

    /*
     * vertex and edge in lego Bundle Adjustment
     */

    // VertexPose: pose vertex
    class VertexPose : public g2o::BaseVertex<6, SE3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void setToOriginImpl() override { _estimate = SE3(Mat44::Identity()); }

        // left multiplication on SE3
        void oplusImpl(const double *update) override {
            Vec6 update_eigen;
            if (std::isnan(update[0]) || std::isnan(update[1]) || std::isnan(update[2]) ||
                std::isnan(update[3]) || std::isnan(update[4]) || std::isnan(update[5]) ||
                std::isinf(update[0]) || std::isinf(update[1]) || std::isinf(update[2]) ||
                std::isinf(update[3]) || std::isinf(update[4]) || std::isinf(update[5]))
                update_eigen.setZero();
            else
                update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate = SE3::exp(update_eigen) * _estimate;
        }

        bool read(std::istream &in) override { return true; }

        bool write(std::ostream &out) const override { return true; }
    };

    // VertexXYZ: landmark point vertex
    class VertexXYZ : public g2o::BaseVertex<3, Vec3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void setToOriginImpl() override { _estimate = Vec3::Zero(); }

        void oplusImpl(const double *update) override {
            if (!std::isnan(update[0]) && !std::isnan(update[1]) && !std::isnan(update[2]) &&
                !std::isinf(update[0]) && !std::isinf(update[1]) && !std::isinf(update[2])) {
                _estimate[0] += update[0];
                _estimate[1] += update[1];
                _estimate[2] += update[2];
            }
        }

        bool read(std::istream &in) override { return true; }

        bool write(std::ostream &out) const override { return true; }
    };

    // EdgeProjectionPoseOnly: unary edge just estimate pose only
    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K) : _pos3d(pos), _K(K) {}

        void computeError() override {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            const SE3 &T = v->estimate();
            Vec3 pos_pixel = _K * (T * _pos3d);
            pos_pixel /= (pos_pixel[2] + 1e-18);
            _error = _measurement - pos_pixel.head<2>();
        }

        void linearizeOplus() override {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            const SE3 &T = v->estimate();
            Vec3 pos_cam = T * _pos3d;

            double fx = _K(0, 0);
            double fy = _K(1, 1);
            // X, Y, Z: the 3D point coordinate in the camera coordinate after transformation
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;

            // jacobian
            _jacobianOplusXi.setZero();
            Eigen::Matrix<double, 2, 6> j_i;
            j_i <<
            -fx*Zinv, 0, fx*X*Zinv2, fx*X*Y*Zinv2, -fx-fx*X*X*Zinv2, fx*Y*Zinv,
            0, -fy*Zinv, fy*Y*Zinv2, fy+fy*Y*Y*Zinv2, -fy*X*Y*Zinv2, -fy*X*Zinv;

            _jacobianOplusXi.block<2, 6>(0, 0) = j_i;
        }

        bool read(std::istream &in) override { return true; }

        bool write(std::ostream &out) const override { return true; }

    private:
        Vec3 _pos3d;
        Mat33 _K;
    };

    // EdgeProjection: edge between pose and landmark point
    class EdgeProjection : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Mat33 &K, const SE3 &cam_ext) : _K(K) {
            _cam_ext = cam_ext;
        }

        void computeError() override {
            const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
            const SE3 &T = v0->estimate();
            Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
            pos_pixel /= (pos_pixel[2] + 1e-18);
            _error = _measurement - pos_pixel.head<2>();
        }

        void linearizeOplus() override {
            const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
            const SE3 &T = v0->estimate();
            const Vec3 &pw = v1->estimate();
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
            _jacobianOplusXi.setZero();
            Eigen::Matrix<double, 2, 6> j_i;
            j_i <<
                -fx*Zinv, 0, fx*X*Zinv2, fx*X*Y*Zinv2, -fx-fx*X*X*Zinv2, fx*Y*Zinv,
                0, -fy*Zinv, fy*Y*Zinv2, fy+fy*Y*Y*Zinv2, -fy*X*Y*Zinv2, -fy*X*Zinv;
            _jacobianOplusXi.block<2, 6>(0, 0) = j_i;

            Eigen::Matrix<double, 2, 3> j_j_part = j_i.block<2, 3>(0, 0);
            _jacobianOplusXj.setZero();
            _jacobianOplusXj.block<2, 3>(0, 0) =  j_j_part * _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        bool read(std::istream &in) override { return true; }

        bool write(std::ostream &out) const override { return true; }

    private:
        Mat33 _K;
        SE3 _cam_ext;
    };
}
#endif  // LEGOSLAM_G2O_TYPES_H
