#include "base_vertex.h"

namespace lego {

    unsigned long global_vertex_id = 0;

    BaseVertex::BaseVertex(int rows, int cols, int dof) {
        estimate_rows_ = rows;
        estimate_cols_ = cols;
        estimate_.resize(rows, cols);
        estimate_backup_.resize(rows, cols);
        dof_ = dof > 0 ? dof : (rows * cols);
        id_ = global_vertex_id++;
    }

    BaseVertex::~BaseVertex() = default;

    unsigned long BaseVertex::getDim() const {
        return estimate_.rows() * estimate_.cols();
    }

    int BaseVertex::getDoF() const {
        return dof_;
    }

    /// linear addition for default
    //void BaseVertex::add(const VecX &delta) { if (estimate_cols_ == 1) estimate_ += delta; }
}
