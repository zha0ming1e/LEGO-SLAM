#ifndef LEGO_BASEEDGE_H
#define LEGO_BASEEDGE_H

#include "eigen3_types.h"
#include "cost_function.h"

namespace lego {

    class BaseVertex;

    /*
     * BaseEdge: residual with vertexes
     *           -> residual = measurement - estimate
     *           -> cost = residual^T * InformationMatrix * residual
     */
    class BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /// constructor
        BaseEdge(int residual_dim, int vertex_num, const std::vector<std::string> &vertex_types=std::vector<std::string>());
        /// destructor
        virtual ~BaseEdge();

        /// set id
        void setId(unsigned long id) { id_ = id; }
        /// get id
        unsigned long getId() const { return id_; }
        /// add vertex
        bool addVertex(const std::shared_ptr<BaseVertex> &vertex) {
            vertexes_.emplace_back(vertex);
            return true;
        }
        /// set vertex
        bool setVertex(const std::vector<std::shared_ptr<BaseVertex>> &vertices) {
            vertexes_ = vertices;
            return true;
        }
        /// get the vertex with index i
        std::shared_ptr<BaseVertex> getVertex(int i) { return vertexes_[i]; }
        /// get all vertexes
        std::vector<std::shared_ptr<BaseVertex>> getAllVertexes() const { return vertexes_; }
        /// get corresponding vertex num
        size_t getVertexNum() const { return vertexes_.size(); }
        /// get edge information: pure virtual function
        virtual std::string getInfo() const = 0;
        /// compute residual
        virtual void computeResidual() = 0;
        /// compute jacobians
        virtual void computeJacobians() = 0;
        /// compute edge factor in hessian matrix
        //virtual void computeHessianFactor() = 0;
        /// compute error^2 and return
        double getChi2() const;
        /// compute error^2 with robust kernel functions and return
        double getRobustChi2() const;
        /// get residual
        VecX getResidual() const { return residual_; }
        /// get residual dimension
        unsigned long getResidualDim() const { return residual_.rows(); }
        /// get jacobians
        std::vector<MatXX> getJacobians() const { return jacobians_; }
        /// set information matrix
        void setInformation(const MatXX &information) {
            information_ = information;
            // sqrt information matrix
            sqrt_information_ = Eigen::LLT<MatXX>(information_).matrixL().transpose();
        }
        /// get information matrix
        MatXX getInformation() const { return information_; }
        /// get sqrt information matrix
        MatXX getSqrtInformation() const { return sqrt_information_; }
        /// set cost function
        void setCostFunction(CostFunction *cost_ptr) { cost_function_ = cost_ptr; }
        /// get cost function
        CostFunction *getCostFunction() { return cost_function_; }
        /// compute robust information matrix
        void computeRobustInformation(double &drho, MatXX &info) const;
        /// set measurement
        void setMeasurement(const VecX &measurement) { measurement_ = measurement; }
        /// get measurement
        VecX getMeasurement() const { return measurement_; }
        /// check valid
        bool checkValid();
        /// get ordering id
        int getOrderingId() const { return ordering_id_; }
        /// set ordering id
        void setOrderingId(int id) { ordering_id_ = id; }

    protected:
        unsigned long id_ = -1;
        int ordering_id_ = -1;
        std::vector<std::string> vertex_types_;
        std::vector<std::shared_ptr<BaseVertex>> vertexes_;
        VecX residual_;
        /// jacobians: dimension = residual X vertex[i]
        std::vector<MatXX> jacobians_;
        /// information matrix
        MatXX information_;
        MatXX sqrt_information_;
        VecX measurement_;
        /// cost function
        CostFunction *cost_function_ = nullptr;
    };
}
#endif  // LEGO_BASEEDGE_H
