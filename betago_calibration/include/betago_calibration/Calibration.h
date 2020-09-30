//
// Created by wang on 20-7-25.
//

#ifndef BETAGO_CALIBRATION_CALIBRATION_H
#define BETAGO_CALIBRATION_CALIBRATION_H

#include <td_ros/tool.h>
#include <ros/package.h>
#include <TdLibrary/slam_tool/motion_transformation.h>
#include <TdLibrary/tool/random_tool.hpp>
#include <gazebo/physics/physics.hh>
class Calibration {
public:
    Calibration(const std::string& calibr_board_name){
        calibr_board_name_ = calibr_board_name;
        pose_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    }
    void SpawnCalibrBoard(){
        std::string sdf_path = ros::package::getPath("betago_calibration") +"/kalibr_tag/model.sdf";
        tdros::SpawnModel(nh_,sdf_path,calibr_board_name_);
    }
    void SetMultiplePoseofCalibrBoard(){
        
//        gazebo::physics::WorldPtr world= gazebo::physics::get_world("default");
//        gazebo::physics::ModelPtr model = world->GetModel(calibr_board_name_);
//        if(!model){
//            ROS_ERROR("no %s exist in gazebo",calibr_board_name_.c_str());
//        }
        //z:0.4-0.64,取0.5
        //pitch 0,-0.67,0 3.14,-1.5708,3.14 3.14,-1.16,3.14
//        std::vector<double> translate_euler = {5.54,-0.02,0.5, 0.0,-0.67,0.0};
        std::vector<double> translate_euler = {5.54,-0.02,0.5, 3.14,-1.5708,3.14};
        Sophus::SE3d pose =  td::EulerTranslatetoSE3(translate_euler);
        Eigen::Vector3d euler1 = {0.0,-0.67,0.0};
        Eigen::Vector3d euler2 = {3.14,-1.5708,3.14};
        Eigen::Vector3d euler3 = {3.14,-1.16,3.14};
        Eigen::Matrix3d R1 = td::EulerToRotation(euler1);
        Eigen::Matrix3d R2 = td::EulerToRotation(euler2);
        Eigen::Matrix3d R3 = td::EulerToRotation(euler3);
        Eigen::Matrix3d R21 = R2.transpose() * R1;
        Eigen::Matrix3d R23 = R2.transpose() * R3;
        Eigen::Vector3d euler21 = td::RotationToEulerAngle(R21);
        Eigen::Vector3d euler23 = td::RotationToEulerAngle(R23);
        std::cout<<euler21.transpose()<<" "<<euler23.transpose()<<std::endl;
        std::vector<double> min_max(2);

        min_max[0] = euler21[1];
        min_max[1] = euler23[1];
        std::cout<<"Random choose"<<std::endl;
        Eigen::Vector3d _euler2n = euler21;
        for(int i=0;i<10;i++){
            _euler2n[1] = td::UniformSampling(min_max[0],min_max[1]);
            std::cout<<_euler2n[1]<<std::endl;
            Eigen::Matrix3d _R2n = td::EulerToRotation(_euler2n);
            Eigen::Matrix3d _Rn = R2 * _R2n;
            Eigen::Vector3d _eulern = td::RotationToEulerAngle(_Rn);
            std::vector<double> translate_euler_n = translate_euler;
            translate_euler_n[3] = _eulern[0];
            translate_euler_n[4] = _eulern[1];
            translate_euler_n[5] = _eulern[2];
            Sophus::SE3d posen =  td::EulerTranslatetoSE3(translate_euler_n);
            tdros::SetPosition(pose_pub_,calibr_board_name_,posen,60,80);
            ros::Duration(1).sleep();

        }


//        Eigen::Matrix3d _R21 = td::EulerToRotation(euler21);
//        Eigen::Matrix3d _R23 = td::EulerToRotation(euler23);
//        Eigen::Matrix3d _Rn = R2 * _R21;
//        Eigen::Vector3d _eulern = td::RotationToEulerAngle(_Rn);
//        std::cout<<_eulern<<std::endl;
//        tdros::SetPosition(pose_pub_,calibr_board_name_,pose,60,80);
    }
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    std::string calibr_board_name_;
};


#endif //BETAGO_CALIBRATION_CALIBRATION_H
