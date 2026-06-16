#include "certifiable_slam_core/RosbagParser.hpp"

#include <iostream>
#include <memory>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

RosbagParser::RosbagParser(const std::string& bag_path) 
    : bag_path_(bag_path) {}

bool RosbagParser::buildOdometryGraph(gtsam::NonlinearFactorGraph& graph, gtsam::Values& initial_estimates) {
    using gtsam::symbol_shorthand::X;

    gtsam::Vector6 prior_sigmas, odom_sigmas;
    prior_sigmas << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    odom_sigmas  << 0.05, 0.05, 0.05, 0.1,  0.1,  0.1;

    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);
    auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas);

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        std::cerr << "Failed to open bag file: " << e.what() << std::endl;
        return false;
    }

    rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
    std::unique_ptr<gtsam::Pose3> previous_pose = nullptr;
    uint64_t pose_index = 0;

    while (reader.has_next()) {
        auto bag_message = reader.read_next();

        if (bag_message->topic_name == "/chassis/odom") {
            nav_msgs::msg::Odometry odom_msg;
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&serialized_msg, &odom_msg);

            const auto& pos = odom_msg.pose.pose.position;
            const auto& q = odom_msg.pose.pose.orientation;

            gtsam::Pose3 current_pose(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z), 
                                      gtsam::Point3(pos.x, pos.y, pos.z));

            if (previous_pose == nullptr) {
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(pose_index), current_pose, prior_noise));
                initial_estimates.insert(X(pose_index), current_pose);
            } else {
                gtsam::Pose3 delta_pose = previous_pose->between(current_pose);
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(pose_index - 1), X(pose_index), delta_pose, odom_noise));
                initial_estimates.insert(X(pose_index), current_pose);
            }

            previous_pose = std::make_unique<gtsam::Pose3>(current_pose);
            pose_index++;
        }
    }
    return true;
}