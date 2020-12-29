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
    bool eye_on_hand = false;
    if(!nh.getParam("dual_arm",dual_arm))
        ROS_INFO("can not get dual_arm from parameter server");
    if(!nh.getParam("eye_on_hand",eye_on_hand))
        ROS_INFO("can not get dual_arm from parameter server");

    if(dual_arm){
        ROS_INFO("DUAL ARM");
        std::string planning_group = "manipulator";
        std::vector<double> joint_group_positions = {0.3452,-1.5219,2.5200,-2.3130,-1.7951,-2.3130,-0.6213,1.6571,-2.52,-1.0702,1.9678,1.9678};//left first
        tdros::toJointSpace(planning_group,joint_group_positions);
    } else{
        ROS_INFO("SINGLE ARM");
        std::string planning_group_right = "right_ur_arm";
        std::string planning_group_left = "left_ur_arm";
        if(eye_on_hand){

            std::vector<double> joint_group_positions_right = {-0.6213,1.6571,-2.52,-1.0702,1.9678,1.9678};
            tdros::toJointSpace(planning_group_right,joint_group_positions_right);

            std::vector<double> joint_group_positions_left = {0.0690,0.4143,1.4844,-2.0368,-0.6559,-2.8308};
            tdros::toJointSpace(planning_group_left,joint_group_positions_left);
        }else{

            std::vector<double> joint_group_positions_right = {-0.6213,1.6571,-2.52,-1.0702,1.9678,1.9678};
            tdros::toJointSpace(planning_group_right,joint_group_positions_right);

            std::vector<double> joint_group_positions_left = {0.3452,-1.5219,2.5200,-2.3130,-1.7951,-2.3130};
            tdros::toJointSpace(planning_group_left,joint_group_positions_left);
        }

    }



  ros::shutdown();
  return 0;
}
