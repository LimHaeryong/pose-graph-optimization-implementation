#include <open3d/Open3D.h>

#include "read_g2o.hpp"
#include "pose_graph_optimizer.hpp"

void visualizeVerticesBeforeAndAfterOptimization(std::shared_ptr<open3d::geometry::PointCloud> vertices_before,
                                                 std::shared_ptr<open3d::geometry::PointCloud> vertices_after)
{
    vertices_before->PaintUniformColor({1, 0, 0});
    vertices_after->PaintUniformColor({0, 1, 0});

    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("PGO Result", 1600, 900);
    visualizer.GetRenderOption().SetPointSize(1.0);
    visualizer.GetRenderOption().background_color_ = {0, 0, 0};
    visualizer.AddGeometry(vertices_before);
    visualizer.AddGeometry(vertices_after);

    visualizer.Run();
}

int main(int argc, char **argv)
{
    std::string filename = G2O_FILE_PATH;
    int iteration = 20;
    if (argc >= 2)
    {
        filename = argv[1];
    }
    if(argc == 3)
    {
        iteration = std::stoi(argv[2]);
    }
    auto [vertices, edges] = PGO::IO::readG2o(filename);

    auto vertices_before = std::make_shared<open3d::geometry::PointCloud>();
    vertices_before->points_.reserve(vertices.size());
    for (const auto &vertex : vertices)
    {
        vertices_before->points_.push_back(vertex.second.pose.p);
    }
    optimizePoseGraph(vertices, edges, iteration);
    auto vertices_after = std::make_shared<open3d::geometry::PointCloud>();
    vertices_after->points_.reserve(vertices.size());
    for (const auto &vertex : vertices)
    {
        vertices_after->points_.push_back(vertex.second.pose.p);
    }

    visualizeVerticesBeforeAndAfterOptimization(vertices_before, vertices_after);

    return 0;
}