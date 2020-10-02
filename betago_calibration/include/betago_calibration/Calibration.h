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
        init_euler_ = {3.14,-1.5708,3.14};
        init_pos_ = {5.54,-0.02,0.5};

    }
    void SpawnCalibrBoard(){
        std::string sdf_path = ros::package::getPath("betago_calibration") +"/kalibr_tag/model.sdf";
        SpawnModel(nh_,sdf_path,calibr_board_name_, init_pos_);
    }
    void SetMultiplePoseofCalibrBoard();
    void PutCalibinInitPose();
private:
    /**
     * return the min and max of the incremental euler angle based on euler2 along i axis
     * @param i 0 for Z, 1 for Y, 2 for X
     * @param euler1 bound euler angle
     * @param euler2 base euler angel
     * @param euler3 another bound euler angle
     * @return [min, max]
     */
    std::vector<double> EulerRange(int i, Eigen::Vector3d &euler1, Eigen::Vector3d &euler2, Eigen::Vector3d &euler3);
    void SpawnModel(ros::NodeHandle& nh, const std::string& path, const std::string& model_name, const Eigen::Vector3d& position){
        std::ifstream file;
        file.open(path);
        if(!file){
            std::cout<<"can't open "<<path<<std::endl;
            exit(-1);
        }
        gazebo_msgs::SpawnModel model;
        ros::ServiceClient client_spwn = nh.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                model.request.model_xml+=line;
            }
            file.close();
        }

        model.request.model_name = model_name;
        model.request.initial_pose.position.x = position[0];
        model.request.initial_pose.position.y = position[1];
        model.request.initial_pose.position.z = position[2];
        client_spwn.call(model);
    }
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    std::string calibr_board_name_;
    Eigen::Vector3d init_euler_;
    Eigen::Vector3d init_pos_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


#endif //BETAGO_CALIBRATION_CALIBRATION_H
