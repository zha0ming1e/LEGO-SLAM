#ifndef LEGO_BASEVERTEX_H
#define LEGO_BASEVERTEX_H

#include "eigen3_types.h"

namespace lego {

    extern unsigned long global_vertex_id;

    /*
     * BaseVertex
     */
    class BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /// constructor: if dof == -1, dof == dim
        explicit BaseVertex(int rows, int cols=1, int dof=-1);
        /// desctructor
        virtual ~BaseVertex();

        /// get vertex dimension
        unsigned long getDim() const;
        /// get vertex dof
        int getDoF() const;
        /// get estimate rows and cols
        int getEstimateRows() const { return estimate_rows_; }
        int getEstimateCols() const { return estimate_cols_; }
        /// set vertex id
        void setId(unsigned long id) { id_ = id; }
        /// get vertex id
        unsigned long getId() const { return id_; }
        /// get vertex estimate
        MatXX getEstimate() const { return estimate_; }
        /// get vertex estimate reference
        MatXX &getEstimate() { return estimate_; }
        /// set vertex estimate
        void setEstimate(const MatXX &estimate) { estimate_ = estimate; }
        /// backup estimate_ in estimate_backup_
        void backupEstimate() { estimate_backup_ = estimate_; }
        /// rollback estimate from estimate_backup_
        void rollbackEstimate() { estimate_ = estimate_backup_; }
        /// get vertex information: pure virtual function
        virtual std::string getInfo() const = 0;
        /// get vertex ordering id
        unsigned long getOrderingId() const { return ordering_id_; }
        /// set vertex ordering id
        void setOrderingId(unsigned long id) { ordering_id_ = id; }
        /// fix vertex: true for default
        void setFixed(bool fixed=true) { fixed_ = fixed; }
        /// if the vertex is set fixed
        bool isFixed() const { return fixed_; }

        /// general addition for overloading, linear vector addition for default
        /// pure virtual for overriding
        virtual void add(const VecX &delta) = 0;

    /// protected: for inheriting
    protected:
        /// estimate
        MatXX estimate_;
        /// backup estimate for rollback
        MatXX estimate_backup_;
        /// estimate rows and cols
        int estimate_rows_ = 0;
        int estimate_cols_ = 1;
        /// degree of freedom
        int dof_;
        unsigned long id_;
        /// ordering id is the ordered id in the problem for searching the corresponding hessian blocks
        unsigned long ordering_id_ = 0;
        bool fixed_ = false;
    };
}
#endif  // LEGO_BASEVERTEX_H
