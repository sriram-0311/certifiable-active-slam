#pragma once

#include <string>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

class RosbagParser {
public:
    explicit RosbagParser(const std::string& bag_path);
    bool buildOdometryGraph(gtsam::NonlinearFactorGraph& graph, gtsam::Values& initial_estimate);

private:
    std::string bag_path_;
};