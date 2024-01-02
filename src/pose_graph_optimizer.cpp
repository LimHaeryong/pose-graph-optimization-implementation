#include "pose_graph_optimizer.hpp"

#include <Eigen/Sparse>
#include <omp.h>

#include <iostream>

namespace PGO
{
    void optimizePoseGraph(Vertices &vertices, const Edges &edges, int iteration)
    {
        double computeLinearSystem_elapsed_time = 0.0, solver_elapsed_time = 0.0, pose_update_elapsed_time = 0.0;

        for (int i = 0; i < iteration; ++i)
        {
            std::cout << "iter " << i << " started.\n";

            double computeLinearSystem_start = omp_get_wtime();
            auto [JTJ, JTr] = computeLinearSystem(vertices, edges);
            double computeLinearSystem_end = omp_get_wtime();

            double solver_start = omp_get_wtime();
            Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(JTJ);
            Eigen::VectorXd dx = solver.solve(-JTr);
            double solver_end = omp_get_wtime();

            double pose_update_start = omp_get_wtime();
            for (size_t k = 0; k < vertices.size(); ++k)
            {
                vertices[k].pose = SE3(dx.block<6, 1>(6 * k, 0) - dx.block<6, 1>(0, 0)) * vertices[k].pose;
            }
            double pose_update_end = omp_get_wtime();

            computeLinearSystem_elapsed_time += computeLinearSystem_end - computeLinearSystem_start;
            solver_elapsed_time += solver_end - solver_start;
            pose_update_elapsed_time += pose_update_end - pose_update_start;
        }
        std::cout << "iteration = " << iteration << "\n";
        std::cout << "total computeLinearSystem elapsed time : " << computeLinearSystem_elapsed_time << "(s)\n";
        std::cout << "total solver elapsed time : " << solver_elapsed_time << "(s)\n";
        std::cout << "total pose update elapsed time : " << pose_update_elapsed_time << "(s)\n";
    }

    std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd> computeLinearSystem(const Vertices &vertices, const Edges &edges)
    {
        size_t num_vertices = vertices.size();
        constexpr size_t TRIPLET_STRIDE = 6 * 6 * 4;
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(edges.size() * TRIPLET_STRIDE);
        Eigen::SparseMatrix<double> JTJ(6 * num_vertices, 6 * num_vertices);
        Eigen::VectorXd JTr(6 * num_vertices);
        JTr.setZero();

        size_t num_threads = omp_get_max_threads();

#pragma omp parallel
        {
            std::vector<Eigen::Triplet<double>> triplets_private;
            triplets_private.reserve(edges.size() * TRIPLET_STRIDE / num_threads);
            Eigen::VectorXd JTr_private(6 * num_vertices);
            JTr_private.setZero();

#pragma omp for nowait
            for (size_t i = 0; i < edges.size(); ++i)
            {
                const auto &edge = edges[i];
                auto iter_a = vertices.find(edge.idx_a);
                auto iter_b = vertices.find(edge.idx_b);
                if (iter_a == vertices.cend() || iter_b == vertices.cend())
                {
                    continue;
                }
                const auto &pose_a = iter_a->second.pose;
                const auto &pose_b = iter_b->second.pose;

                auto error = edge.T_ab.inverse() * pose_a.inverse() * pose_b;
                Mat6d jacobian_pose_a = -1.0 * error.rightJacobianInverse() * pose_b.inverse().adjoint();

                Mat6d JTJ_aa = jacobian_pose_a.transpose() * edge.information_matrix * jacobian_pose_a;
                Vec6d JTr_a = jacobian_pose_a.transpose() * edge.information_matrix * error.se3();

                for (int row = 0; row < 6; ++row)
                {
                    for (int col = 0; col < 6; ++col)
                    {
                        triplets_private.emplace_back(edge.idx_a * 6 + row, edge.idx_a * 6 + col, JTJ_aa(row, col));
                        triplets_private.emplace_back(edge.idx_a * 6 + row, edge.idx_b * 6 + col, -JTJ_aa(row, col));
                        triplets_private.emplace_back(edge.idx_b * 6 + row, edge.idx_a * 6 + col, -JTJ_aa(row, col));
                        triplets_private.emplace_back(edge.idx_b * 6 + row, edge.idx_b * 6 + col, JTJ_aa(row, col));
                    }
                }

                JTr_private.block<6, 1>(edge.idx_a * 6, 0) += JTr_a;
                JTr_private.block<6, 1>(edge.idx_b * 6, 0) -= JTr_a;
            }
#pragma omp critical
            {
                triplets.insert(triplets.end(), triplets_private.begin(), triplets_private.end());
                JTr += JTr_private;
            }
        }

        JTJ.setFromTriplets(triplets.begin(), triplets.end());

        return std::make_pair(JTJ, JTr);
    }

}; // namespace PGO