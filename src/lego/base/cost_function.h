#ifndef LEGO_COST_FUNCTION_H
#define LEGO_COST_FUNCTION_H

#include "eigen3_types.h"

namespace lego {

    /*
     * CostFunction
     *
     * rho[0]: the actual scaled error value
     * rho[1]: the first derivative of the scaling function
     * rho[2]: the second derivative of the scaling function
     */
    class CostFunction {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual ~CostFunction() = default;

        // virtual double Compute(double error) const = 0;
        virtual void compute(double err2, Eigen::Vector3d &rho) const = 0;
    };

    /*
     * TrivialCost(x) = x^2
     */
    class TrivialCost : public CostFunction {
    public:
        void compute(double err2, Eigen::Vector3d &rho) const override {
            rho[0] = err2;
            rho[1] = 1;
            rho[2] = 0;
        }
    };

    /*
     *                    --- x^2                if x <= delta
     * HuberCost(x) = ---|
     *                    --- delta*(2*x-delta)  if x > delta
     */
    class HuberCost : public CostFunction {
    public:
        explicit HuberCost(double delta) : delta_(delta) {}

        void compute(double err2, Eigen::Vector3d &rho) const override;

    private:
        double delta_;
    };

    /*
     * CauchyCost(x) = c^2 * log(1 + x / c^2)
     */
    class CauchyCost : public CostFunction {
    public:
        explicit CauchyCost(double delta) : delta_(delta) {}

        void compute(double err2, Eigen::Vector3d &rho) const override;

    private:
        double delta_;
    };

    /*
     * TukeyCost(x)
     */
    class TukeyCost : public CostFunction {
    public:
        explicit TukeyCost(double delta) : delta_(delta) {}

        void compute(double err2, Eigen::Vector3d &rho) const override;

    private:
        double delta_;
    };
}
#endif  // LEGO_COST_FUNCTION_H
