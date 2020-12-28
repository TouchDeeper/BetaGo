//
// Created by wang on 20-12-8.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <td_ros/publisher/gazebo_pose_publisher.hpp>

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "odometry_output_node");
    ros::NodeHandle nh;

    std::shared_ptr<tdros::GazeboPosePublisher> betago_pose_pub_ptr = std::make_shared<tdros::GazeboPosePublisher>(nh, "/gazebo/set_model_state", 10);
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0,3) = 3;
    pose(2,3) = 0.025898;
    betago_pose_pub_ptr->Publish(pose, "ridgeback");

    return 0;
}
