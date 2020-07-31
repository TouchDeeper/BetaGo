//
// Created by wang on 20-7-25.
//

#ifndef BETAGO_CALIBRATION_CALIBRATION_H
#define BETAGO_CALIBRATION_CALIBRATION_H

#include <td_ros/tool.h>
#include <ros/package.h>
#include <TdLibrary/slam_tool/motion_transformation.h>
#include <gazebo/physics/physics.hh>
class Calibration {
public:
    Calibration(const std::string& calibr_board_name){
        calibr_board_name_ = calibr_board_name;
        pose_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    }
    void SpawnCalibrBoard(){
        std::string sdf_path = ros::package::getPath("betago_calibration") +"/kalibr_tag/model.sdf";
        tdros::SpawnModel(nh_,sdf_path,calibr_board_name_);
    }
    void SetMultiplePoseofCalibrBoard(){
        
        gazebo::physics::WorldPtr world= gazebo::physics::get_world("default");
        gazebo::physics::ModelPtr model = world->GetModel(calibr_board_name_);
        if(!model){
            ROS_ERROR("no %s exist in gazebo",calibr_board_name_.c_str());
        }
        std::vector<double> euler_translate = {0.0,M_PI_2,0,5,0,0};
        Sophus::SE3d pose =  td::EulerTranslatetoSE3(euler_translate);
        tdros::SetPosition(pose_pub_,calibr_board_name_,pose,60);
    }
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    std::string calibr_board_name_;
};


#endif //BETAGO_CALIBRATION_CALIBRATION_H
