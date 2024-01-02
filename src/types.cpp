#include "types.hpp"
#include "math.hpp"


namespace PGO
{
    SE3::SE3(const Vec6d &se3)
    {
        Eigen::AngleAxisd aa(se3.tail(3).norm(), se3.tail(3).normalized());
        this->q = Eigen::Quaterniond(aa);

        Eigen::Matrix3d J = this->computeJ();
        this->p = J * se3.head(3);        
    }

    SE3 SE3::inverse() const
    {
        SE3 inv;
        inv.q = this->q.conjugate();
        inv.p = -1.0 * ( inv.q * this->p);
        return inv;
    }

    Mat6d SE3::adjoint() const
    {
        Eigen::Matrix3d R = this->q.toRotationMatrix();
        
        Mat6d adj = Mat6d::Zero();
        adj.block<3, 3>(0, 0) = R;
        adj.block<3, 3>(3, 3) = R;
        adj.block<3, 3>(0, 3) = Math::skewSymmetric(this->p) * R;

        return adj;
    }

    Vec6d SE3::se3() const
    {
        Eigen::AngleAxisd aa(this->q);
        Eigen::Matrix3d J = this->computeJ();
        Vec6d se3;

        se3.head(3) = J.inverse() * p;
        se3.tail(3) = aa.angle() * aa.axis();
        return se3;
    }

    Mat6d SE3::rightJacobianInverse() const
    {
        Mat6d output = Mat6d::Identity();
        Vec6d se3 = this->se3();
        output.block<3, 3>(0, 0).noalias() += 0.5 * Math::skewSymmetric(se3.tail(3));
        output.block<3, 3>(3, 3) = output.block<3, 3>(0, 0);
        output.block<3, 3>(0, 3).noalias() += 0.5 * Math::skewSymmetric(se3.head(3));
        return output;
    }

    Eigen::Matrix3d SE3::computeJ() const
    {
        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
        Eigen::AngleAxisd aa(this->q);
        if(aa.angle() < 1e-6)
        {
            J.setIdentity();
        }
        else
        {
            double factor_1 = std::sin(aa.angle()) / aa.angle();
            double factor_2 = (1.0 - std::cos(aa.angle())) / aa.angle();
            J.noalias() += factor_1 * Eigen::Matrix3d::Identity() + (1.0 - factor_1) * aa.axis() * aa.axis().transpose() + factor_2 * Math::skewSymmetric(aa.axis());
        }

        return J;
    }

    SE3& SE3::operator*=(const SE3& other)
    {
        this->q *= other.q;
        this->p += this->q * other.p;
        return *this;
    }

    SE3 SE3::operator*(const SE3& other) const
    {
        SE3 se3;
        se3.q = this->q * other.q;
        se3.p = this->p + this->q * other.p;
        return se3;
    }

    std::istream &operator>>(std::istream &is, SE3 &se3)
    {
        is >> se3.p.x() >> se3.p.y() >> se3.p.z() >>
            se3.q.x() >> se3.q.y() >> se3.q.z() >> se3.q.w();
        se3.q.normalize();
        return is;
    }

    std::istream &operator>>(std::istream &is, Vertex &vertex)
    {
        is >> vertex.idx >> vertex.pose;
        return is;
    }

    std::istream &operator>>(std::istream &is, Edge &constraint)
    {
        is >> constraint.idx_a >> constraint.idx_b >> constraint.T_ab;

        for (int i = 0; i < 6; ++i)
        {
            for (int j = i; j < 6; ++j)
            {
                double value;
                is >> value;
                constraint.information_matrix(i, j) = value;
                constraint.information_matrix(j, i) = value;
            }
        }

        return is;
    }

    

}; // namespace PGO