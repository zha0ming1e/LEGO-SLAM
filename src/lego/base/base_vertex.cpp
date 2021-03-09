#include "base_vertex.h"

namespace lego {

    unsigned long global_vertex_id = 0;

    BaseVertex::BaseVertex(int dim, int local_dim) {
        estimate_.resize(dim, 1);
        local_dimension_ = local_dim > 0 ? local_dim : dim;
        id_ = global_vertex_id++;
    }

    BaseVertex::~BaseVertex() = default;

    int BaseVertex::getDim() const {
        return estimate_.rows();
    }

    int BaseVertex::getLocalDim() const {
        return local_dimension_;
    }

    void BaseVertex::add(const VecX &delta) {
        /// linear vector addition for default
        estimate_ += delta;
    }
}
