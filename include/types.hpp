#ifndef PGO_TYPES_HPP_
#define PGO_TYPES_HPP_

#include <unordered_map>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

namespace PGO
{
    using VertexIndex = uint32_t;
    using Mat6d = typename Eigen::Matrix<double, 6, 6>;
    using Vec6d = typename Eigen::Vector<double, 6>;

    struct SE3
    {
        Eigen::Vector3d p;
        Eigen::Quaterniond q;

        SE3() {}

        SE3(const Vec6d &se3);

        SE3 inverse() const;
        Mat6d adjoint() const;
        Vec6d se3() const;
        Mat6d rightJacobianInverse() const;
        Eigen::Matrix3d computeJ() const;

        SE3 &operator*=(const SE3 &other);
        SE3 operator*(const SE3 &other) const;
    };

    struct Vertex
    {
        VertexIndex idx;
        SE3 pose;
    };
    using Vertices = typename std::unordered_map<VertexIndex, Vertex>;

    struct Edge
    {
        VertexIndex idx_a, idx_b;

        SE3 T_ab;
        Eigen::Matrix<double, 6, 6> information_matrix;
    };
    using Edges = typename std::vector<Edge>;

    std::istream &operator>>(std::istream &is, SE3 &se3);
    std::istream &operator>>(std::istream &is, Vertex &vertex);
    std::istream &operator>>(std::istream &is, Edge &constraint);

}; // namespace PGO

#endif // PGO_TYPES_HPP_