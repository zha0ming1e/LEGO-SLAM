#include "cost_function.h"

namespace lego {

    void HuberCost::compute(double err2, Eigen::Vector3d &rho) const {
        double d2 = delta_ * delta_;
        if (err2 <= d2) { // inlier
            rho[0] = err2;
            rho[1] = 1.0;
            rho[2] = 0.0;
        } else { // outlier
            double sqrte = sqrt(err2); // absolut value of the error
            rho[0] = 2*sqrte*delta_ - d2; // rho(e)   = 2 * delta * e^(1/2) - delta^2
            rho[1] = delta_ / sqrte;        // rho'(e)  = delta / sqrt(e)
            rho[2] = -0.5 * rho[1] / err2;    // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
        }
    }

    void CauchyCost::compute(double err2, Eigen::Vector3d &rho) const {
        double d2 = delta_ * delta_;       // c^2
        double d2Reci = 1.0 / d2;         // 1/c^2
        double aux = d2Reci * err2 + 1.0;  // 1 + e^2/c^2
        rho[0] = d2 * std::log(aux);            // c^2 * log( 1 + e^2/c^2 )
        rho[1] = 1.0 / aux;                   // rho'
        rho[2] = -d2Reci * std::pow(rho[1], 2); // rho''
    }

    void TukeyCost::compute(double err2, Eigen::Vector3d &rho) const {
        const double e = sqrt(err2);
        const double d2 = delta_ * delta_;
        if (e <= delta_) {
            const double aux = err2 / d2;
            rho[0] = d2 * (1.0 - std::pow((1.0 - aux), 3)) / 3.0;
            rho[1] = std::pow((1.0 - aux), 2);
            rho[2] = -2.0 * (1.0 - aux) / d2;
        } else {
            rho[0] = d2 / 3.0;
            rho[1] = 0;
            rho[2] = 0;
        }
    }
}
