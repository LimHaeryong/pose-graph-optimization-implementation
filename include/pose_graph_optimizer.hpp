#ifndef PGO_POSE_GRAPH_OPTIMIZER_HPP_
#define PGO_POSE_GRAPH_OPTIMIZER_HPP_

#include "types.hpp"
#include <Eigen/Sparse>

namespace PGO
{
    void optimizePoseGraph(Vertices & vertices, const Edges & edges, int iteration);
    std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd> computeLinearSystem(const Vertices & vertices, const Edges & edges);
} // namespace PGO



#endif // PGO_POSE_GRAPH_OPTIMIZER_HPP_