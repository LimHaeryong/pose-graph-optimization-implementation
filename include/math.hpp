#ifndef PGO_MATH_HPP_
#define PGO_MATH_HPP_

#include <Eigen/Dense>

namespace PGO::Math
{
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &vec);
    Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d &R);
    Eigen::Matrix3d rotationVectorToMatrix(const Eigen::Vector3d &r);
    Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d &r);
    Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond &quat);

} // namespace PGO

#endif // PGO_MATH_HPP_