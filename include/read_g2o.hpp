#ifndef PGO_READ_G2O_HPP_
#define PGO_READ_G2O_HPP_

#include "types.hpp"

namespace PGO::IO
{
    std::pair<Vertices, Edges> readG2o(const std::string& filename);

}; // namespace PGO::IO

#endif // PGO_READ_G2O_HPP_