#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <td_ros/arm_moveit.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "to_default_pose");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  bool dual_arm = false;
  if(!nh.getParam("dual_arm",dual_arm))
      ROS_INFO("can not get dual_arm from parameter server");

  if(dual_arm){
      ROS_INFO("DUAL ARM");
      std::string planning_group = "manipulator";
      std::vector<double> joint_group_positions = {0,-1.38,2.58,-2.76,-1.62,-3.14,3.14,1.27,-2.52,-1.58,1.76,0.0};//left first
      tdros::toJointSpace(planning_group,joint_group_positions);
  } else{
      ROS_INFO("SINGLE ARM");
      std::string planning_group_right = "right_ur_arm";
      std::vector<double> joint_group_positions_right = {3.14,1.27,-2.52,-1.58,1.76,0.0};
      tdros::toJointSpace(planning_group_right,joint_group_positions_right);

      std::string planning_group_left = "left_ur_arm";
      std::vector<double> joint_group_positions_left = {0,-1.38,2.58,-2.76,-1.62,-3.14};
      tdros::toJointSpace(planning_group_left,joint_group_positions_left);
  }



  ros::shutdown();
  return 0;
}
