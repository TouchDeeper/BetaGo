//
// Created by wang on 20-7-24.
//
#include <td_ros/tool.h>
#include <betago_calibration/Calibration.h>
#include <gazebo/Server.hh>
int main(int argc, char** argv){
    ros::init(argc,argv,"cam_laser_calibr_node");

    Calibration calibr("kalibr_tag");
    calibr.SetMultiplePoseofCalibrBoard();
//    calibr.SpawnCalibrBoard();
    ros::spin();
    return 0;
}
