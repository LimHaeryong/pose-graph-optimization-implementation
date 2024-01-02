#include "math.hpp"

namespace PGO::Math
{
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d skew;
        skew << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;
        return skew;
    }

    Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d &R)
    {
        Eigen::AngleAxisd aa(R);
        return aa.angle() * aa.axis();
    }

    Eigen::Matrix3d rotationVectorToMatrix(const Eigen::Vector3d &r)
    {
        Eigen::AngleAxisd aa(r.norm(), r.normalized());
        return aa.toRotationMatrix();
    }

    Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d &r)
    {
        Eigen::AngleAxisd aa(r.norm(), r.normalized());
        return Eigen::Quaterniond(aa);
    }

    Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond & quat)
    {
        Eigen::AngleAxisd aa(quat);
        return aa.angle() * aa.axis();
    }
    
}; // namespace PGO::Math
