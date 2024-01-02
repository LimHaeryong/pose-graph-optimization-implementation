#include "read_g2o.hpp"

namespace PGO::IO
{
    std::pair<Vertices, Edges> readG2o(const std::string &filename)
    {
        Vertices vertices;
        Edges edges;

        std::ifstream file_stream(filename);

        std::string type;
        while (file_stream.good())
        {
            file_stream >> type;
            if (type == "VERTEX_SE3:QUAT")
            {
                Vertex vertex;
                file_stream >> vertex;
                vertices[vertex.idx] = vertex;
            }
            else if (type == "EDGE_SE3:QUAT")
            {
                Edge edge;
                file_stream >> edge;
                edges.push_back(edge);
            }

            file_stream >> std::ws;
        }

        return std::make_pair(vertices, edges);
    }
}; // namespace PGO::IO
