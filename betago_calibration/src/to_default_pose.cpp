#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "TdRosLib/arm_moveit.h"
int main(int argc, char** argv)
{
    // TODO trajectory line visual problem
  ros::init(argc, argv, "to_default_pose");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

//    std::string planning_group_right = "right_ur_arm";
//    std::vector<double> joint_group_positions_right = {3.14,1.27,-2.52,-1.58,1.76,0.0};
//    toJointSpace(planning_group_right,joint_group_positions_right);
//
//    std::string planning_group_left = "left_ur_arm";
//    std::vector<double> joint_group_positions_left = {0,-1.20,2.31,0,0,0.0};
//    toJointSpace(planning_group_left,joint_group_positions_left);

    std::string planning_group = "manipulator";
    std::vector<double> joint_group_positions = {0,-1.20,2.31,0,0,0.0,3.14,1.27,-2.52,-1.58,1.76,0.0};//left first
    tdros::toJointSpace(planning_group,joint_group_positions);
  ros::shutdown();
  return 0;
}