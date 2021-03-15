#include "problem.h"
#include "timer.h"

#ifdef USE_OPENMP
#include <omp.h>
#endif

/// utils
/// define the format
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

/// write to csv files
void writeToCSVfile(const std::string &name, const Eigen::MatrixXd &matrix) {
    std::ofstream f(name.c_str());
    f << matrix.format(CSVFormat);
}

namespace lego {

    void Problem::logoutVectorSize() {
         //LOG(INFO) << "Vertexes = " << vertexes_.size()
         //          << ", Edges = " << edges_.size();
    }

    Problem::Problem(ProblemType problemType, AlgorithmType algorithmType, StrategyType strategyType)
            : problemType_(problemType), algorithmType_(algorithmType), strategyType_(strategyType) {
        /// initialization
        logoutVectorSize();
        vertexes_marg_.clear();
    }

    Problem::~Problem() { global_vertex_id = 0; }

    bool Problem::addVertex(const std::shared_ptr<BaseVertex> &vertex) {
        if (vertexes_.find(vertex->getId()) != vertexes_.end()) {
            //LOG(WARNING) << "Vertex: " << vertex->getId() << " has been added before. ";
            return false;
        } else {
            vertexes_.insert(std::pair<unsigned long, std::shared_ptr<BaseVertex>>(vertex->getId(), vertex));
        }

        if (problemType_ == ProblemType::SLAM) {
            if (isPoseVertex(vertex)) resizePoseHessiansWhenAddingPose(vertex);
        }
        return true;
    }

    void Problem::addOrderingSLAM(const std::shared_ptr<BaseVertex> &v) {
        if (isPoseVertex(v)) {
            v->setOrderingId(ordering_poses_);
            idx_pose_vertexes_.insert(std::pair<ulong, std::shared_ptr<BaseVertex>>(v->getId(), v));
            ordering_poses_ += v->getDoF();
        } else if (isLandmarkVertex(v)) {
            v->setOrderingId(ordering_landmarks_);
            ordering_landmarks_ += v->getDoF();
            idx_landmark_vertexes_.insert(std::pair<ulong, std::shared_ptr<BaseVertex>>(v->getId(), v));
        }
    }

    void Problem::resizePoseHessiansWhenAddingPose(const std::shared_ptr<BaseVertex> &v) {
        int size = H_prior_.rows() + v->getDoF();
        H_prior_.conservativeResize(size, size);
        b_prior_.conservativeResize(size);

        b_prior_.tail(v->getDoF()).setZero();
        H_prior_.rightCols(v->getDoF()).setZero();
        H_prior_.bottomRows(v->getDoF()).setZero();
    }

    void Problem::extendHessiansPriorSize(int dim) {
        int size = H_prior_.rows() + dim;
        H_prior_.conservativeResize(size, size);
        b_prior_.conservativeResize(size);

        b_prior_.tail(dim).setZero();
        H_prior_.rightCols(dim).setZero();
        H_prior_.bottomRows(dim).setZero();
    }

    bool Problem::isPoseVertex(const std::shared_ptr<BaseVertex> &v) {
        std::string type = v->getInfo();
        return type == std::string("VertexPose") ||
               type == std::string("VertexSpeedBias");
    }

    bool Problem::isLandmarkVertex(const std::shared_ptr<BaseVertex> &v) {
        std::string type = v->getInfo();
        return type == std::string("VertexXYZ") ||
               type == std::string("VertexPointXYZ") ||
               type == std::string("VertexInverseDepth");
    }

    bool Problem::addEdge(const std::shared_ptr<BaseEdge> &edge) {
        if (edges_.find(edge->getId()) == edges_.end()) {
            edges_.insert(std::pair<ulong, std::shared_ptr<BaseEdge>>(edge->getId(), edge));
        } else {
            //LOG(WARNING) << "Edge: " << edge->getId() << " has been added before. ";
            return false;
        }

        for (auto &vertex : edge->getAllVertexes())
            vertexToEdge_.insert(std::pair<ulong, std::shared_ptr<BaseEdge>>(vertex->getId(), edge));

        return true;
    }

    std::vector<std::shared_ptr<BaseEdge>> Problem::getConnectedEdges(const std::shared_ptr<BaseVertex> &vertex) {
        std::vector<std::shared_ptr<BaseEdge>> edges;
        auto range = vertexToEdge_.equal_range(vertex->getId());
        for (auto iter = range.first; iter != range.second; ++iter) {
            /// the edge is needed in the future rather than has been removed
            if (edges_.find(iter->second->getId()) == edges_.end()) continue;

            edges.emplace_back(iter->second);
        }

        return edges;
    }

    bool Problem::removeVertex(const std::shared_ptr<BaseVertex> &vertex) {
        /// check if the vertex is in map_vertexes_
        if (vertexes_.find(vertex->getId()) == vertexes_.end()) {
            //LOG(WARNING) << "The vertex: " << vertex->getId() << " is not in the problem. " << std::endl;
            return false;
        }

        /// remove edges corresponding to the vertex
        std::vector<std::shared_ptr<BaseEdge>> remove_edges = getConnectedEdges(vertex);
        for (auto &remove_edge : remove_edges) {
            removeEdge(remove_edge);
        }

        if (isPoseVertex(vertex))
            idx_pose_vertexes_.erase(vertex->getId());
        else
            idx_landmark_vertexes_.erase(vertex->getId());

        vertex->setOrderingId(-1);      // used to debug
        vertexes_.erase(vertex->getId());
        vertexToEdge_.erase(vertex->getId());

        return true;
    }

    bool Problem::removeEdge(std::shared_ptr<BaseEdge> edge) {
        /// check if the edge is in map_edges_
        if (edges_.find(edge->getId()) == edges_.end()) {
            //LOG(WARNING) << "The edge: " << edge->getId() << " is not in the problem. " << std::endl;
            return false;
        }
        edges_.erase(edge->getId());

        return true;
    }

    bool Problem::solve(int iterations) {
        if (edges_.empty() || vertexes_.empty()) {
            std::cerr << "\nError: Cannot solve problem without vertexes or edges. " << std::endl;
            std::cerr << "Please check vertexes and edges in the problem. " << std::endl;
            return false;
        }

        if (verbose_) std::cout << "==========LEGO OPTIMIZER==========" << std::endl;

        Timer t_cost;
        /// calculate the dimension of the problem for building hessian matrix
        setOrdering();
        /// build the hessian matrix
        buildHessian();

        /// Levenberg-Marquardt algorithm
        /// initialization of LM
        computeLambdaInitLM();
        /// LM solver
        bool stop = false;
        int iter = 0;
        double last_chi_ = 1e20;
        const int false_cnt_threshold = 10;
        while (!stop && (iter < iterations)) {
            if (verbose_) {
                std::cout << "Iteration = " << iter
                          << ",\tChi = " << currentChi_
                          << ",\tLambda = " << currentLambda_ << std::endl;
            }

            bool oneStepSuccess = false;
            int false_cnt = 0;
            /// try Lambda until a successful iteration step
            while (!oneStepSuccess && (false_cnt < false_cnt_threshold)) {
                /// add lambda to hessian matrix and solve linear system
                solveLinearEquation();

                /// update states
                updateStates();
                /// is a good step in LM and update the lambda
                oneStepSuccess = isGoodStepInLM();
                if (oneStepSuccess) {
                    /// build hessian matrix in the new linearization point
                    buildHessian();
                    false_cnt = 0;
                } else {
                    false_cnt ++;
                    /// is not a good step in LM, then rollback
                    rollbackStates();
                }
            }
            ++iter;

            /// quit the optimization: chi difference between two steps < diffChiThreshold_
            if (last_chi_ - currentChi_ < diffChiThreshold_) {
                if (verbose_) {
                    std::cout << "\nStop the optimization: "
                              << "[last_chi_(" << last_chi_ << ") - currentChi_(" << currentChi_ << ") = "
                              << last_chi_ - currentChi_
                              << "] < " << diffChiThreshold_ << std::endl;
                }
                stop = true;
            }
            last_chi_ = currentChi_;
        }

        double tc = t_cost.toc();
        if (verbose_) {
            std::cout << "\nInfo: \nTimeCost(SolveProblem) = " << tc << " ms" << std::endl;
            std::cout << "TimeCost(BuildHessian) = " << t_hessian_cost_ << " ms" << std::endl;
        }
        t_hessian_cost_ = 0.0;

        return true;
    }

    //bool Problem::solveBASEProblem(int iterations) { return true; }

    void Problem::setOrdering() {
        /// counting
        ordering_poses_ = 0;
        ordering_generic_ = 0;
        ordering_landmarks_ = 0;

        for (auto &vertex : vertexes_) {
            /// dimension if all vertexes
            ordering_generic_ += vertex.second->getDoF();

            /// count vertex and edge respectively
            if (problemType_ == ProblemType::SLAM)
                addOrderingSLAM(vertex.second);
        }

        if (problemType_ == ProblemType::SLAM) {
            /// landmark vertex is behind pose vertex
            ulong all_pose_dimension = ordering_poses_;
            for (auto &landmarkVertex : idx_landmark_vertexes_)
                landmarkVertex.second->setOrderingId(landmarkVertex.second->getOrderingId() + all_pose_dimension);
        }
    }

    bool Problem::checkOrdering() {
        if (problemType_ == ProblemType::SLAM) {
            int current_ordering = 0;
            for (auto &v : idx_pose_vertexes_) {
                assert(v.second->getOrderingId() == current_ordering);
                current_ordering += v.second->getDoF();
            }

            for (auto &v : idx_landmark_vertexes_) {
                assert(v.second->getOrderingId() == current_ordering);
                current_ordering += v.second->getDoF();
            }
        }
        return true;
    }

    void Problem::buildHessian() {
        Timer t_h;

        /// build the hessian matrix
        ulong size = ordering_generic_;
        MatXX H(MatXX::Zero(size, size));
        VecX b(VecX::Zero(size));

/// openmp for accelerating
#ifdef USE_OPENMP
#pragma omp parallel for num_threads(4)
#endif
        for (auto &edge : edges_) {
            /// compute residual and jacobian
            edge.second->computeResidual();
            edge.second->computeJacobians();

            /// build the hessian with all blocks
            auto jacobians = edge.second->getJacobians();
            auto vertexes = edge.second->getAllVertexes();
            assert(jacobians.size() == vertexes.size());
            for (size_t i = 0; i < vertexes.size(); ++i) {
                auto v_i = vertexes[i];
                /// fixed: jacobian == 0
                if (v_i->isFixed()) continue;

                auto jacobian_i = jacobians[i];
                ulong index_i = v_i->getOrderingId();
                ulong dim_i = v_i->getDoF();

                /// trivial or robust cost kernel function
                double drho;
                MatXX robustInfo(edge.second->getInformation().rows(), edge.second->getInformation().cols());
                edge.second->computeRobustInformation(drho, robustInfo);

                /// jacobian * RobustInformationMatrix
                MatXX JtW = jacobian_i.transpose() * robustInfo;
                for (size_t j = i; j < vertexes.size(); ++j) {
                    auto v_j = vertexes[j];

                    if (v_j->isFixed()) continue;

                    auto jacobian_j = jacobians[j];
                    ulong index_j = v_j->getOrderingId();
                    ulong dim_j = v_j->getDoF();

                    assert(v_j->getOrderingId() != -1);
                    MatXX hessian = JtW * jacobian_j;

                    /// accumulate hessian blocks
                    H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                    if (j != i) {
                        /// non-diag blocks
                        H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                    }
                }
                b.segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose() * edge.second->getInformation() * edge.second->getResidual();
            }
        }

        Hessian_ = H;
        b_ = b;
        t_hessian_cost_ += t_h.toc();

        /// add prior information to the hessian
        if(H_prior_.rows() > 0) {
            MatXX H_prior_tmp = H_prior_;
            VecX b_prior_tmp = b_prior_;

            /// set the prior of pose vertex to 0, landmark vertex has no prior
            /// fix prior
            for (auto &vertex : vertexes_) {
                if (isPoseVertex(vertex.second) && vertex.second->isFixed()) {
                    size_t idx = vertex.second->getOrderingId();
                    size_t dim = vertex.second->getDoF();
                    H_prior_tmp.block(idx, 0, dim, H_prior_tmp.cols()).setZero();
                    H_prior_tmp.block(0, idx, H_prior_tmp.rows(), dim).setZero();
                    b_prior_tmp.segment(idx, dim).setZero();
                }
            }
            Hessian_.topLeftCorner(ordering_poses_, ordering_poses_) += H_prior_tmp;
            b_.head(ordering_poses_) += b_prior_tmp;
        }
        /// initialize delta_x_ to 0 vector
        delta_x_ = VecX::Zero(size);
    }

    /// solve the linear equation
    /// sparse Cholesky solver or PCG solver
    void Problem::solveLinearEquation() {
        if (problemType_ == ProblemType::BASE) {
            MatXX H = Hessian_;
            for (long i = 0; i < Hessian_.cols(); ++i) {
                /// strategy
                if (strategyType_ == StrategyType::DEFAULT) {
                    /// default strategy
                    H(i, i) += currentLambda_;
                }
                else if (strategyType_ == StrategyType::STRATEGY1) {
                    /// strategy 1
                    H(i, i) += currentLambda_ * H(i, i);
                }
            }
            /// Cholesky decomposition
            delta_x_ = H.ldlt().solve(b_);
            /// PCG solver
            //delta_x_ = PCGSolver(H, b_, H.rows() * 2);
        } else {
            /// step 1: schur complement for marginalization -> Hpp, bpp
            int reserve_size = ordering_poses_;
            int marg_size = ordering_landmarks_;
            MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
            MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
            MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
            VecX bpp = b_.segment(0, reserve_size);
            VecX bmm = b_.segment(reserve_size, marg_size);

            /// Hmm is a diag matrix, acceleration for its inverse
            MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
/// openmp for accelerating
#ifdef USE_OPENMP
#pragma omp parallel for num_threads(4)
#endif
            for (auto &landmarkVertex : idx_landmark_vertexes_) {
                ulong idx = landmarkVertex.second->getOrderingId() - reserve_size;
                ulong size = landmarkVertex.second->getDoF();
                Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
            }

            MatXX tempH = Hpm * Hmm_inv;
            H_pp_schur_ = Hessian_.block(0, 0, ordering_poses_, ordering_poses_) - tempH * Hmp;
            b_pp_schur_ = bpp - tempH * bmm;

            /// step 2: solve Hpp * delta_x = bpp
            VecX delta_x_pp(VecX::Zero(reserve_size));
            for (ulong i = 0; i < ordering_poses_; ++i) {
                /// strategy
                if (strategyType_ == StrategyType::DEFAULT) {
                    /// default strategy
                    H_pp_schur_(i, i) += currentLambda_;
                }
                else if (strategyType_ == StrategyType::STRATEGY1) {
                    /// strategy 1
                    H_pp_schur_(i, i) += currentLambda_ * H_pp_schur_(i, i);
                }
            }
            /// Cholesky decomposition
            delta_x_pp =  H_pp_schur_.ldlt().solve(b_pp_schur_);
            /// PCG solver
            //delta_x_pp = PCGSolver(H_pp_schur_, b_pp_schur_, H_pp_schur_.rows() * 2);
            /// update
            delta_x_.head(reserve_size) = delta_x_pp;

            /// step 3: solve Hmm * delta_x = bmm - Hmp * delta_x_pp
            VecX delta_x_ll(marg_size);
            delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
            delta_x_.tail(marg_size) = delta_x_ll;
        }
    }

    void Problem::updateStates() {
        /// update vertexes
        for (auto &vertex : vertexes_) {
            /// backup estimate in the last iteration
            vertex.second->backupEstimate();

            ulong idx = vertex.second->getOrderingId();
            ulong dim = vertex.second->getDoF();
            VecX delta = delta_x_.segment(idx, dim);

            vertex.second->add(delta);
        }

        /// update the prior
        if (err_prior_.rows() > 0) {
            /// backup prior in the last iteration
            b_prior_backup_ = b_prior_;
            err_prior_backup_ = err_prior_;
            /// update prior with the first order Taylor expansion
            b_prior_ -= H_prior_ * delta_x_.head(ordering_poses_);
            err_prior_ = -Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 15);
        }
    }

    void Problem::rollbackStates() {
        /// rollback vertexes
        for (auto &vertex: vertexes_)
            vertex.second->rollbackEstimate();

        /// rollback prior
        if (err_prior_.rows() > 0) {
            b_prior_ = b_prior_backup_;
            err_prior_ = err_prior_backup_;
        }
    }

    /// initial lambda in LM
    void Problem::computeLambdaInitLM() {
        ni_ = 2.0;
        currentLambda_ = -1.0;
        currentChi_ = 0.0;

        for (auto &edge : edges_)
            currentChi_ += edge.second->getRobustChi2();
        if (err_prior_.rows() > 0)
            currentChi_ += err_prior_.squaredNorm();
        currentChi_ *= 0.5;

        /// stop threshold of iteration
        stopThresholdLM_ = 1e-10 * currentChi_;
        /// strategy
        if (strategyType_ == StrategyType::DEFAULT) {
            /// default strategy
            double maxDiagonal = 0;
            ulong size = Hessian_.cols();
            assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square matrix. ");
            /// is set the initial lambda
            if (!isSetInitialLambda_) {
                for (ulong i = 0; i < size; ++i)
                    maxDiagonal = std::max(std::abs(Hessian_(i, i)), maxDiagonal);

                maxDiagonal = std::min(5e10, maxDiagonal);
                double tau = 1e-5;
                currentLambda_ = tau * maxDiagonal;
            } else {
                currentLambda_ = initialLambda_;
            }
        } else if (strategyType_ == StrategyType::STRATEGY1) {
            /// strategy 1
            currentLambda_ = 1e-5;
        }
    }

    void Problem::addLambdaToHessianLM() {
        ulong size = Hessian_.cols();
        assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square matrix. ");
        for (ulong i = 0; i < size; ++i)
            Hessian_(i, i) += currentLambda_;
    }

    void Problem::removeLambdaHessianLM() {
        ulong size = Hessian_.cols();
        assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square matrix. ");
        for (ulong i = 0; i < size; ++i)
            Hessian_(i, i) -= currentLambda_;
    }

    bool Problem::isGoodStepInLM() {
        /// recompute residuals after state updated
        double tempChi = 0.0;
        for (auto &edge : edges_) {
            edge.second->computeResidual();
            tempChi += edge.second->getRobustChi2();
        }
        if (err_prior_.size() > 0)
            tempChi += err_prior_.squaredNorm();
        tempChi *= 0.5;

        /// strategy
        if (strategyType_ == StrategyType::DEFAULT) {
            /// default strategy
            double scale = 0;
            scale = 0.5 * delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
            scale += 1e-10;

            double rho = (currentChi_ - tempChi) / scale;
            /// rho > 0: cost is decreasing
            if (rho > 0 && std::isfinite(tempChi)) {
                double alpha = 1.0 - std::pow((2 * rho - 1), 3);
                alpha = std::min(alpha, 2.0 / 3.0);
                double scaleFactor = std::max(1.0 / 3.0, alpha);
                currentLambda_ *= scaleFactor;
                ni_ = 2;
                currentChi_ = tempChi;

                return true;
            } else {
                currentLambda_ *= ni_;
                ni_ *= 2;

                return false;
            }
        } else if (strategyType_ == StrategyType::STRATEGY1) {
            /// strategy 1
            ulong size = delta_x_.rows();
            // diag(Hessian_)
            MatXX diag_Hessian(MatXX::Zero(size, size));
            for (ulong i = 0; i < size; ++i)
                diag_Hessian(i, i) = Hessian_(i, i);
            // scale: denominator of rho
            double scale = 0;
            scale = 0.5 * delta_x_.transpose() * (currentLambda_ * diag_Hessian * delta_x_ + b_);
            scale += 1e-10;

            double rho = (currentChi_ - tempChi) / scale;
            /// rho > 0: cost is decreasing
            double L_down = 9.0, L_up = 11.0;
            if (rho > 0 && std::isfinite(tempChi)) {
                currentLambda_ = std::max(currentLambda_ / L_down, 1e-7);
                currentChi_ = tempChi;

                return true;
            } else {
                currentLambda_ = std::min(currentLambda_ * L_up, 1e7);

                return false;
            }
        }
    }

    /// PCGSolver: conjugate gradient with precondition
    VecX Problem::PCGSolver(const MatXX &A, const VecX &b, int maxIter=-1) {
        assert(A.rows() == A.cols() && "PCGSolver Error: A is not a square matrix in eq Ax = b");
        int rows = b.rows();
        int n = maxIter < 0 ? rows : maxIter;
        VecX x(VecX::Zero(rows));
        MatXX M_inv = A.diagonal().asDiagonal().inverse();
        VecX r0(b);  // initial r = b - A*0 = b
        VecX z0 = M_inv * r0;
        VecX p(z0);
        VecX w = A * p;
        double r0z0 = r0.dot(z0);
        double alpha = r0z0 / p.dot(w);
        VecX r1 = r0 - alpha * w;
        int i = 0;
        double threshold = 1e-6 * r0.norm();
        while (r1.norm() > threshold && i < n) {
            ++i;
            VecX z1 = M_inv * r1;
            double r1z1 = r1.dot(z1);
            double belta = r1z1 / r0z0;
            z0 = z1;
            r0z0 = r1z1;
            r0 = r1;
            p = belta * p + z1;
            w = A * p;
            alpha = r1z1 / p.dot(w);
            x += alpha * p;
            r1 -= alpha * w;
        }
        return x;
    }

    /// marginalize edges connecting to pose vertexes: reprojection factor and imu factor
    bool Problem::marginalize(const std::vector<std::shared_ptr<BaseVertex>> &margVertexes, int pose_dim) {
        setOrdering();
        /// edges need to be marginalized
        std::vector<std::shared_ptr<BaseEdge>> marg_edges = getConnectedEdges(margVertexes[0]);

        std::unordered_map<int, std::shared_ptr<BaseVertex>> margLandmark;
        /// keep pose vertex ordering, reorder landmark vertex ordering
        int marg_landmark_size = 0;
        for (auto &marg_edge : marg_edges) {
            auto vertexes = marg_edge->getAllVertexes();
            for (auto &iter : vertexes) {
                if (isLandmarkVertex(iter) && margLandmark.find(iter->getId()) == margLandmark.end()) {
                    iter->setOrderingId(pose_dim + marg_landmark_size);
                    margLandmark.insert(std::make_pair(iter->getId(), iter));
                    marg_landmark_size += iter->getDoF();
                }
            }
        }

        int cols = pose_dim + marg_landmark_size;
        /// hessian: H = H_marg + H_pp_prior
        MatXX H_marg(MatXX::Zero(cols, cols));
        VecX b_marg(VecX::Zero(cols));
        /// edge factor count
        int ii = 0;
        for (auto &edge: marg_edges) {
            edge->computeResidual();
            edge->computeJacobians();

            auto jacobians = edge->getJacobians();
            auto vertexes = edge->getAllVertexes();
            ii++;

            assert(jacobians.size() == vertexes.size());
            for (size_t i = 0; i < vertexes.size(); ++i) {
                auto v_i = vertexes[i];
                auto jacobian_i = jacobians[i];
                ulong index_i = v_i->getOrderingId();
                ulong dim_i = v_i->getDoF();

                double drho;
                MatXX robustInfo(edge->getInformation().rows(), edge->getInformation().cols());
                edge->computeRobustInformation(drho, robustInfo);
                for (size_t j = i; j < vertexes.size(); ++j) {
                    auto v_j = vertexes[j];
                    auto jacobian_j = jacobians[j];
                    ulong index_j = v_j->getOrderingId();
                    ulong dim_j = v_j->getDoF();

                    MatXX hessian = jacobian_i.transpose() * robustInfo * jacobian_j;
                    assert(hessian.rows() == v_i->getDoF() && hessian.cols() == v_j->getDoF());

                    H_marg.block(index_i, index_j, dim_i, dim_j) += hessian;
                    if (j != i) {
                        /// non-diag blocks
                        H_marg.block(index_j, index_i, dim_j, dim_i) += hessian.transpose();
                    }
                }
                b_marg.segment(index_i, dim_i) -= drho * jacobian_i.transpose() * edge->getInformation() * edge->getResidual();
            }
        }
        if (verbose_) std::cout << "Maginalization: Edge factor count = " << ii <<std::endl;

        /// marginalize landmark
        int reserve_size = pose_dim;
        if (marg_landmark_size > 0) {
            int marg_size = marg_landmark_size;
            MatXX Hmm = H_marg.block(reserve_size, reserve_size, marg_size, marg_size);
            MatXX Hpm = H_marg.block(0, reserve_size, reserve_size, marg_size);
            MatXX Hmp = H_marg.block(reserve_size, 0, marg_size, reserve_size);
            VecX bpp = b_marg.segment(0, reserve_size);
            VecX bmm = b_marg.segment(reserve_size, marg_size);

            /// Hmm is a diag matrix, acceleration for its inverse
            MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
/// openmp for accelerating
#ifdef USE_OPENMP
#pragma omp parallel for num_threads(4)
#endif
            for (auto &iter : margLandmark) {
                ulong idx = iter.second->getOrderingId() - reserve_size;
                ulong size = iter.second->getDoF();
                Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
            }

            MatXX tempH = Hpm * Hmm_inv;
            MatXX Hpp = H_marg.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;
            bpp = bpp - tempH * bmm;
            H_marg = Hpp;
            b_marg = bpp;
        }

        VecX b_prior_before = b_prior_;
        if (H_prior_.rows() > 0) {
            H_marg += H_prior_;
            b_marg += b_prior_;
        }

        /// marginalize pose vertexes and speed bias vertexes
        int marg_dim = 0;
        /// move matrix blocks
        for (ulong k = margVertexes.size() - 1 ; k >= 0; --k) {
            ulong idx = margVertexes[k]->getOrderingId();
            ulong dim = margVertexes[k]->getDoF();
            marg_dim += dim;

            /// move the marginalized pose vertex blocks to the bottom right of Hmm
            Eigen::MatrixXd temp_rows = H_marg.block(idx, 0, dim, reserve_size);
            Eigen::MatrixXd temp_botRows = H_marg.block(idx + dim, 0, reserve_size - idx - dim, reserve_size);
            H_marg.block(idx, 0, reserve_size - idx - dim, reserve_size) = temp_botRows;
            H_marg.block(reserve_size - dim, 0, dim, reserve_size) = temp_rows;

            Eigen::MatrixXd temp_cols = H_marg.block(0, idx, reserve_size, dim);
            Eigen::MatrixXd temp_rightCols = H_marg.block(0, idx + dim, reserve_size, reserve_size - idx - dim);
            H_marg.block(0, idx, reserve_size, reserve_size - idx - dim) = temp_rightCols;
            H_marg.block(0, reserve_size - dim, reserve_size, dim) = temp_cols;

            Eigen::VectorXd temp_b = b_marg.segment(idx, dim);
            Eigen::VectorXd temp_btail = b_marg.segment(idx + dim, reserve_size - idx - dim);
            b_marg.segment(idx, reserve_size - idx - dim) = temp_btail;
            b_marg.segment(reserve_size - dim, dim) = temp_b;
        }

        /// threshold
        double eps = 1e-8;
        int m2 = marg_dim;
        int n2 = reserve_size - marg_dim;
        Eigen::MatrixXd Amm = 0.5 * (H_marg.block(n2, n2, m2, m2) + H_marg.block(n2, n2, m2, m2).transpose());

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
        Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd(
                (saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() *
                                  saes.eigenvectors().transpose();

        Eigen::VectorXd bmm2 = b_marg.segment(n2, m2);
        Eigen::MatrixXd Arm = H_marg.block(0, n2, n2, m2);
        Eigen::MatrixXd Amr = H_marg.block(n2, 0, m2, n2);
        Eigen::MatrixXd Arr = H_marg.block(0, 0, n2, n2);
        Eigen::VectorXd brr = b_marg.segment(0, n2);
        Eigen::MatrixXd tempB = Arm * Amm_inv;
        H_prior_ = Arr - tempB * Amr;
        b_prior_ = brr - tempB * bmm2;

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_prior_);
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
        Jt_prior_inv_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        err_prior_ = -Jt_prior_inv_ * b_prior_;

        MatXX J = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        H_prior_ = J.transpose() * J;
        MatXX tmp_h = MatXX((H_prior_.array().abs() > 1e-9).select(H_prior_.array(), 0));
        H_prior_ = tmp_h;

        /// remove the vertexes and edges
        for (auto &margVertex : margVertexes)
            removeVertex(margVertex);
        for (auto &landmarkVertex : margLandmark)
            removeVertex(landmarkVertex.second);

        return true;
    }
}
