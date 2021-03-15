#include "base_vertex.h"
#include "base_edge.h"

namespace lego {

    unsigned long global_edge_id = 0;

    BaseEdge::BaseEdge(int residual_dim, int vertex_num,
                       int measurement_rows, int measurement_cols,
                       const std::vector<std::string> &vertex_types) {
        /// residual
        residual_.resize(residual_dim, 1);
        /// measurement
        measurement_.resize(measurement_rows, measurement_cols);
        /// vertex types
        if (!vertex_types.empty()) vertex_types_ = vertex_types;
        /// jacobians
        jacobians_.resize(vertex_num);
        id_ = global_edge_id++;

        Eigen::MatrixXd information(residual_dim, residual_dim);
        information.setIdentity();
        information_ = information;

        /// cost function initialization
        //cost_function_ = nullptr;
    }

    BaseEdge::~BaseEdge() = default;

    double BaseEdge::getChi2() const { return residual_.transpose() * information_ * residual_; }

    double BaseEdge::getRobustChi2() const {
        double e2 = this->getChi2();
        if (cost_function_) {
            Eigen::Vector3d rho;
            cost_function_->compute(e2, rho);
            e2 = rho[0];
        }

        return e2;
    }

    void BaseEdge::computeRobustInformation(double &drho, MatXX &info) const {
        if (cost_function_) {
            /// robust_info = rho[1] * information_ + information_ * r * r^T * information_
            double e2 = this->getChi2();
            Eigen::Vector3d rho;
            cost_function_->compute(e2, rho);
            VecX weight_err = information_ * residual_;

            MatXX robust_info(information_.rows(), information_.cols());
            robust_info.setIdentity();
            robust_info *= rho[1] * information_;
            if (rho[1] + 2*rho[2]*e2 > 0.0)
                robust_info += 2 * rho[2] * weight_err * weight_err.transpose();

            info = robust_info;
            drho = rho[1];
        } else {
            drho = 1.0;
            info = information_;
        }
    }

    bool BaseEdge::checkValid() {
        if (!vertex_types_.empty()) {
            /// check type info
            for (size_t i = 0; i < vertexes_.size(); ++i) {
                if (vertex_types_[i] != vertexes_[i]->getInfo()) {
                    std::cout << "Vertex type does not match, should be "
                              << vertex_types_[i]
                              << ", but set to " << vertexes_[i]->getInfo() << std::endl;
                    return false;
                }
            }
        }
        return true;
    }
}
