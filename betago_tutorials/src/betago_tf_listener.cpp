//
// Created by wang on 20-6-28.
//
#include <ros/ros.h>
#include <tf/transform_listener.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

//    ros::service::waitForService("spawn");
//    ros::ServiceClient add_turtle =
//            node.serviceClient<turtlesim::Spawn>("spawn");
//    turtlesim::Spawn srv;
//    add_turtle.call(srv);

//    ros::Publisher turtle_vel =
//            node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/right_ur_arm_base", "/right_ur_arm_tool0",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        auto base_q_tool = transform.getRotation();
        ROS_INFO("q.x:%.2f, q.y:%.2f, q.z:%.2f,q.w:%.2f",base_q_tool.x(),base_q_tool.y(),base_q_tool.z(),base_q_tool.w());
        auto base_t_tool = transform.getOrigin();
        ROS_INFO("x:%.2f, y:%.2f, z:%.2f",base_t_tool.x(),base_t_tool.y(),base_t_tool.z());
//        turtlesim::Velocity vel_msg;
//        vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
//                                      transform.getOrigin().x());
//        vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
//                                    pow(transform.getOrigin().y(), 2));
//        turtle_vel.publish(vel_msg);

        rate.sleep();
    }
    return 0;
};