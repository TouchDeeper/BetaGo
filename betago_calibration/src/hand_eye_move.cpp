/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <TdLibrary/FileOperation/file_manager.hpp>
#include <TdLibrary/FileOperation/FileOperation.h>

void saveWaypoint(moveit::planning_interface::MoveGroupInterface& move_group, std::string& waypoint_path){
    std::ofstream ofs;

    ofs.open(waypoint_path.c_str(), std::ios::app);
    if (!ofs) {
        std::cout << "无法生成文件: " << std::endl << waypoint_path << std::endl << std::endl;
    }
    geometry_msgs::Pose now_pose = move_group.getCurrentPose().pose;
    ofs<<now_pose.position.x<<" "<<now_pose.position.y<<" "<<now_pose.position.z<<" "
       <<now_pose.orientation.x<<" "<<now_pose.orientation.y<<" "<<now_pose.orientation.z<<" "<<now_pose.orientation.w<<std::endl;
    ofs.close();
    ros::spin();
}
void saveJointspace(moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group , std::string& waypoint_path){
    std::ofstream ofs;

    ofs.open(waypoint_path.c_str(), std::ios::app);
    if (!ofs) {
        std::cout << "无法生成文件: " << std::endl << waypoint_path << std::endl << std::endl;
    }
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    std::vector<std::string> joint_group = move_group.getActiveJoints();
    assert(joint_group.size() == joint_group_positions.size());
//    joint_model_group->printGroupInfo();
    for (int j = 0; j < joint_group_positions.size(); ++j) {
        std::cout<<joint_group[j]<<" angle: "<<joint_group_positions[j]<<std::endl;
        ofs<<joint_group_positions[j]<<" ";
    }
    ofs<<std::endl;
    ofs.close();
    ros::spin();
}
void addWaypoint(std::vector<geometry_msgs::Pose>& waypoints, std::string& waypoint_path){
    std::ifstream ifs;
    ifs.open(waypoint_path.c_str(),std::ios::in);
    if(!ifs)
    {
        std::cout<<"can't find"<<waypoint_path <<std::endl;
        exit(-1);
    }
    std::string line;
    geometry_msgs::Pose waypoint;
    while (std::getline(ifs, line) && !line.empty()) {
        std::istringstream line_data(line);
        line_data >> waypoint.position.x >> waypoint.position.y >> waypoint.position.z >> waypoint.orientation.x >> waypoint.orientation.y >>
                  waypoint.orientation.z>>waypoint.orientation.w;
        waypoints.push_back(waypoint);
    }
}
void addJointspace(std::vector<std::vector<double>>& waypoints, std::string& joint_space_path){
    std::ifstream ifs;
    ifs.open(joint_space_path.c_str(),std::ios::in);
    if(!ifs)
    {
        std::cout<<"can't find"<<joint_space_path <<std::endl;
        exit(-1);
    }
    std::string line;
    std::vector<double> joint_group_position(6);
    while (std::getline(ifs, line) && !line.empty()) {
        std::istringstream line_data(line);
        for (int i = 0; i < 6; ++i) {
            line_data >> joint_group_position[i];
        }
        waypoints.push_back(joint_group_position);
    }
}
void toJointspace(moveit::planning_interface::MoveGroupInterface& move_group, moveit_visual_tools::MoveItVisualTools& visual_tools, std::vector<double>& joint_group_positions, Eigen::Affine3d& text_pose, const robot_state::JointModelGroup* joint_model_group){
    move_group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    move_group.move();
//     Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}
void toTarget(geometry_msgs::Pose& target_pose, moveit::planning_interface::MoveGroupInterface& move_group, moveit_visual_tools::MoveItVisualTools& visual_tools, Eigen::Affine3d& text_pose, const robot_state::JointModelGroup* joint_model_group){

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    move_group.clearPoseTargets();
    move_group.setPoseTarget(target_pose);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
//    std::vector<geometry_msgs::Pose> waypoints;
//    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
//    waypoints.push_back(current_pose);
//    waypoints.push_back(target_pose);
//    moveit_msgs::RobotTrajectory trajectory;
//    const double jump_threshold = 0.0;
//    const double eef_step = 0.01;
//    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
//    my_plan.trajectory_ = trajectory;

    // Visualize the plan in RViz
//    visual_tools.deleteAllMarkers();
//    visual_tools.publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//    visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
//    for (std::size_t i = 0; i < waypoints.size(); ++i)
//        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose");
    visual_tools.publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    move_group.execute(my_plan);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

}
void toWaypoints(moveit::planning_interface::MoveGroupInterface& move_group, moveit_visual_tools::MoveItVisualTools& visual_tools,std::string& waypoint_path, Eigen::Affine3d& text_pose){
    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    addWaypoint(waypoints, waypoint_path);

    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
//    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("tutorial", "Visualizing path %s", success ? "" : "FAILED");

    move_group.execute(my_plan);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

}
int main(int argc, char** argv)
{
    // TODO trajectory line visual problem
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "left_ur_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setGoalTolerance(0.01);
    move_group.setMaxAccelerationScalingFactor(1);
    move_group.setMaxVelocityScalingFactor(1);
    move_group.setNumPlanningAttempts(10);
    move_group.setPlanningTime(5);
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  std::string reference = move_group.getPlanningFrame();//get model's root frame
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", reference.c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(reference);
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//    visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.


//  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_group_positions;
//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//
//  std::vector<std::string> joint_group = move_group.getActiveJoints();
//  assert(joint_group.size() == joint_group_positions.size());
////    joint_model_group->printGroupInfo();
//    for (int j = 0; j < joint_group_positions.size(); ++j) {
//        std::cout<<joint_group[j]<<" angle: "<<joint_group_positions[j]<<std::endl;
//    }
//  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//  joint_group_positions[0] = -1.0;  // radians
//  move_group.setJointValueTarget(joint_group_positions);
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    //Jointspace path
    std::string joint_space_path = ros::package::getPath("betago_calibration") + "/calib_raw_data/hand_eye/joint_space_path.txt";
//    saveJointspace(move_group,joint_model_group,joint_space_path);
    std::vector<std::vector<double>> joint_space_waypoint;
    addJointspace(joint_space_waypoint, joint_space_path);
    for (int j = 0; j < joint_space_waypoint.size(); ++j) {
        for (int i = 0; i < 6; ++i) {
            std::cout<<joint_space_waypoint[j][i]<<" ";
        }
        std::cout<<std::endl;
        toJointspace(move_group,visual_tools,joint_space_waypoint[j],text_pose,joint_model_group);
    }

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
//    std::string waypoint_path = ros::package::getPath("betago_calibration") + "/calib_raw_data/hand_eye/path.txt";
//    std::vector<geometry_msgs::Pose> waypoints;
//    addWaypoint(waypoints, waypoint_path);
//    for (int i = 0; i < waypoints.size(); ++i) {
//        ROS_INFO("pose %lf %lf %lf %lf %lf %lf %lf", waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z,
//                waypoints[i].orientation.x, waypoints[i].orientation.y, waypoints[i].orientation.z, waypoints[i].orientation.w);
//        toTarget(waypoints[i],move_group,visual_tools,text_pose,joint_model_group);
//    }
//    toWaypoints(move_group,visual_tools,waypoint_path,text_pose);




//  // Adding/Removing Objects and Attaching/Detaching Objects
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // Define a collision object ROS message.
//  moveit_msgs::CollisionObject collision_object;
//  collision_object.header.frame_id = move_group.getPlanningFrame();
//
//  // The id of the object is used to identify it.
//  collision_object.id = "box1";
//
//  // Define a box to add to the world.
//  shape_msgs::SolidPrimitive primitive;
//  primitive.type = primitive.BOX;
//  primitive.dimensions.resize(3);
//  primitive.dimensions[0] = 0.4;
//  primitive.dimensions[1] = 0.1;
//  primitive.dimensions[2] = 0.4;
//
//  // Define a pose for the box (specified relative to frame_id)
//  geometry_msgs::Pose box_pose;
//  box_pose.orientation.w = 1.0;
//  box_pose.position.x = 0.4;
//  box_pose.position.y = -0.4;
//  box_pose.position.z = 1.0;
//
//  collision_object.primitives.push_back(primitive);
//  collision_object.primitive_poses.push_back(box_pose);
//  collision_object.operation = collision_object.ADD;
//
//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  collision_objects.push_back(collision_object);
//
//  // Now, let's add the collision object into the world
//  ROS_INFO_NAMED("tutorial", "Add an object into the world");
//  planning_scene_interface.addCollisionObjects(collision_objects);
//
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  // Wait for MoveGroup to recieve and process the collision object message
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
//
//  // Now when we plan a trajectory it will avoid the obstacle
//  move_group.setStartState(*move_group.getCurrentState());
//  geometry_msgs::Pose another_pose;
//  another_pose.orientation.x = 0.5;
//  another_pose.orientation.y = -0.5;
//  another_pose.orientation.z = -0.5;
//  another_pose.orientation.w = -0.5;
//  another_pose.position.x = 0.3688;
//  another_pose.position.y = -0.602;
//  another_pose.position.z = 0.972;
//  move_group.setPoseTarget(another_pose);
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("next step");
//
//  // Now, let's attach the collision object to the robot.
//  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
//  move_group.attachObject(collision_object.id);
//
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  /* Wait for MoveGroup to recieve and process the attached collision object message */
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
//                      "robot");
//
//  // Now, let's detach the collision object from the robot.
//  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
//  move_group.detachObject(collision_object.id);
//
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  /* Wait for MoveGroup to recieve and process the attached collision object message */
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
//                      "robot");
//
//  // Now, let's remove the collision object from the world.
//  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
//  std::vector<std::string> object_ids;
//  object_ids.push_back(collision_object.id);
//  planning_scene_interface.removeCollisionObjects(object_ids);
//
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  /* Wait for MoveGroup to recieve and process the attached collision object message */
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
//
//  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
