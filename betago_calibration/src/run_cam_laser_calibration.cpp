//
// Created by wang on 20-7-24.
//
#include <td_ros/tool.h>
#include <betago_calibration/Calibration.h>
#include <gazebo/Server.hh>
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("receive image");
//    bag_.write("/head_camera/rgb/image_raw",ros::Time::now(),*msg);
}
int main(int argc, char** argv){
    ros::init(argc,argv,"cam_laser_calibr_node");
    ros::NodeHandle nh("~");
    Calibration calibr("kalibr_tag", nh);
    calibr.PutCalibinInitPose();
    calibr.SetMultiplePoseofCalibrBoard();
//    ros::Subscriber sub_image = nh.subscribe("/head_camera/rgb/image_raw", 1, ImageCallback);//you must hold on the sub object until you want to unsubscribe.
//    calibr.SpawnCalibrBoard();
//    ros::spin();
    return 0;
}
