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
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <rosbag/bag.h>
#include <TdLibrary/FileOperation/file_manager.hpp>
#include <td_ros/tf_listener/tf_listener.hpp>
class Calibration {
public:
    Calibration(const std::string& calibr_board_name, ros::NodeHandle &nh);
    void SpawnCalibrBoard(){
        std::string sdf_path = ros::package::getPath("betago_calibration") +"/kalibr_tag/model.sdf";
        SpawnModel(nh_,sdf_path,calibr_board_name_, init_pos_);
    }
    void SetMultiplePoseofCalibrBoard();
    void PutCalibinInitPose();
    static void imageCallback(const sensor_msgs::ImageConstPtr& msg, bool& flag);
    static void scanCallback(const sensor_msgs::LaserScanConstPtr& msg, bool& flag);
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
    void recordBag(ros::NodeHandle &nh, const std::string& topic, rosbag::Bag& bag);
    void initData();
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    std::string calibr_board_name_;
    Eigen::Vector3d init_euler_;
    Eigen::Vector3d init_pos_;
    static rosbag::Bag bag_;
    std::string topic_name_;
    std::string calib_raw_data_path_;
    static std::string mode_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


#endif //BETAGO_CALIBRATION_CALIBRATION_H
