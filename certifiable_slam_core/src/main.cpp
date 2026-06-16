#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "certifiable_slam_core/RosbagParser.hpp"

int main() {
    // Relative path to your dataset folder (ensure you run the executable from where this folder lives)
    std::string bag_path = "warehouse_slam_dataset_01";
    
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimates;

    RosbagParser parser(bag_path);

    std::cout << "Starting offline parse of: " << bag_path << "..." << std::endl;

    if (!parser.buildOdometryGraph(graph, initial_estimates)) {
        std::cerr << "Pipeline aborted due to parser failure." << std::endl;
        return 1;
    }

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Graph Construction Complete!" << std::endl;
    std::cout << "Total Spatial Factors Added: " << graph.size() << std::endl;
    std::cout << "Total State Vertices Tracked: " << initial_estimates.size() << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    return 0;
}