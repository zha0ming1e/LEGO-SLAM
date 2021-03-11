#ifndef LEGO_PROBLEM_H
#define LEGO_PROBLEM_H

#include "eigen3_types.h"
#include "base_vertex.h"
#include "base_edge.h"

namespace lego {

    typedef unsigned long ulong;
    //typedef std::unordered_map<unsigned long, std::shared_ptr<BaseVertex>> HashVertex;
    typedef std::map<unsigned long, std::shared_ptr<BaseVertex>> VertexMap;
    typedef std::unordered_map<unsigned long, std::shared_ptr<BaseEdge>> EdgeMap;
    typedef std::unordered_multimap<unsigned long, std::shared_ptr<BaseEdge>> VertexIdToEdge;

    /*
     * Problem
     */
    class Problem {
    public:
        /*
         * ProblemType: BASE or SLAM
         *              |
         *              | --- BASE: general non-linear optimisation -> dense
         *              | --- SLAM: SLAM problem with marginalisation -> sparse
         */
        enum class ProblemType {
            /// BASE: general problems
            BASE,
            /// SLAM: SLAM problems
            SLAM
        };

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /// constructor
        explicit Problem(ProblemType problemType);
        /// destructor
        ~Problem();

        /// add vertex
        bool addVertex(const std::shared_ptr<BaseVertex> &vertex);
        /// remove a vertex
        bool removeVertex(const std::shared_ptr<BaseVertex> &vertex);
        /// add edge
        bool addEdge(const std::shared_ptr<BaseEdge> &edge);
        /// remove edge
        bool removeEdge(std::shared_ptr<BaseEdge> edge);
        /// get all vertexes
        VertexMap getAllVertexes() const { return vertexes_; }
        /// get all edges
        EdgeMap getAllEdges() const { return edges_; }

        /// get outliers for frontend to remove these edges
        void getOutlierEdges(std::vector<std::shared_ptr<BaseEdge>> &outlier_edges);

        /// solve the problem
        bool solve(int iterations=10);

        /// marginalization
        bool marginalize(std::shared_ptr<BaseVertex> &margVertexes, const std::vector<std::shared_ptr<BaseVertex>> &landmarkVertexes);
        bool marginalize(const std::shared_ptr<BaseVertex> &margVertexes);
        bool marginalize(const std::vector<std::shared_ptr<BaseVertex>> &margVertexes, int pose_dim);

        /// get data members
        MatXX getHessianPrior() { return H_prior_; }
        VecX getbPrior() { return b_prior_; }
        VecX getErrPrior() { return err_prior_; }
        MatXX getJtPrior() { return Jt_prior_inv_; }
        /// set data members
        void setHessianPrior(const MatXX &H) { H_prior_ = H; }
        void setbPrior(const VecX &b) { b_prior_ = b; }
        void setErrPrior(const VecX &b) { err_prior_ = b; }
        void setJtPrior(const MatXX &J) { Jt_prior_inv_ = J; }
        /// set initial lambda: must before the solve() function
        void setInitialLambda(double initLambda) {
            isSetInitialLambda_ = true;
            initialLambda_ = initLambda;
        }
        /// set verbose
        void setVerbose(bool verb) { verbose_ = verb; }

        /// extend hessian prior matrix size to fit new data
        void extendHessiansPriorSize(int dim);
        /// test computing prior
        //void testComputePrior();

    private:
        /// function member
        /// solve base problem
        //bool solveBASEProblem(int iterations);
        /// solve slam problem
        //bool solveSLAMProblem(int iterations);

        /// set ordering_index of every vertex
        void setOrdering();
        /// set ordering for new vertex in slam problem
        void addOrderingSLAM(const std::shared_ptr<BaseVertex> &v);
        /// build the hessian matrix
        void buildHessian();
        /// schur complement for SBA
        void schurSBA();
        /// solve linear equation
        void solveLinearEquation();
        /// update states
        void updateStates();
        /// rollback states
        void rollbackStates();
        /// compute prior and update it
        void computePrior();

        bool isPoseVertex(const std::shared_ptr<BaseVertex> &v);
        bool isLandmarkVertex(const std::shared_ptr<BaseVertex> &v);
        void resizePoseHessiansWhenAddingPose(const std::shared_ptr<BaseVertex> &v);
        bool checkOrdering();
        void logoutVectorSize();

        /// get edges of a vertex
        std::vector<std::shared_ptr<BaseEdge>> getConnectedEdges(const std::shared_ptr<BaseVertex> &vertex);

        /// Levenberg-Marquardt algorithm
        /// compute initial lambda
        void computeLambdaInitLM();
        /// add or remove lambda to the diag of hessian
        void addLambdaToHessianLM();
        void removeLambdaHessianLM();
        /// the strategy of update the lambda in LM
        bool isGoodStepInLM();

        /// PCG solver
        VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);

        /// data member
        double currentLambda_;
        double initialLambda_;
        bool isSetInitialLambda_ = false;
        double currentChi_;
        double stopThresholdLM_;
        /// the coe to control lambda
        double ni_;

        ProblemType problemType_;

        /// hessian
        MatXX Hessian_;
        VecX b_;
        VecX delta_x_;

        /// prior
        MatXX H_prior_;
        VecX b_prior_;
        VecX b_prior_backup_;
        VecX err_prior_backup_;

        MatXX Jt_prior_inv_;
        VecX err_prior_;

        /// schur pose block in SBA
        MatXX H_pp_schur_;
        VecX b_pp_schur_;
        // landmark and pose block in hessian
        MatXX H_pp_;
        VecX b_pp_;
        MatXX H_ll_;
        VecX b_ll_;

        /// all vertexes
        VertexMap vertexes_;
        /// all edges
        EdgeMap edges_;

        /// vertex id to edge
        VertexIdToEdge vertexToEdge_;

        /// ordering related
        ulong ordering_poses_ = 0;
        ulong ordering_landmarks_ = 0;
        ulong ordering_generic_ = 0;
        /// pose vertexes ordered by ordering id
        std::map<unsigned long, std::shared_ptr<BaseVertex>> idx_pose_vertexes_;
        /// landmark vertexes ordered by ordering id
        std::map<unsigned long, std::shared_ptr<BaseVertex>> idx_landmark_vertexes_;

        // vertexes need to be marginalized
        VertexMap vertexes_marg_;

        bool bDebug = false;
        double t_hessian_cost_ = 0.0;
        double t_PCGsovle_cost_ = 0.0;

        // is verbose or not
        bool verbose_ = true;
    };
}
#endif  // LEGO_PROBLEM_H
