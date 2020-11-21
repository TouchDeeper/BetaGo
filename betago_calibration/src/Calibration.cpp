//
// Created by wang on 20-7-25.
//

#include "betago_calibration/Calibration.h"
#include <td_ros/tf_listener/tf_listener.hpp>
#include "betago_calibration/utilities.h"
rosbag::Bag Calibration::bag_;
std::string Calibration::mode_ = "cam_laser";

Calibration::Calibration(const std::string& calibr_board_name, ros::NodeHandle &nh):nh_(nh){
    calibr_board_name_ = calibr_board_name;
    pose_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    init_euler_ = {3.14,-1.5708,3.14};
    init_pos_ = {4.54,-2.02,0.5};
    if(!nh_.getParam("mode",mode_))
        ROS_INFO("did not set the mode");
    ROS_INFO("RUN MODE: %s", mode_.c_str());
    initData();


}
void Calibration::initData() {
    calib_raw_data_path_ = ros::package::getPath("betago_calibration");
    if(mode_ == "intrinsic")
        calib_raw_data_path_ += "/calib_raw_data/intrinsic/";
    else
        calib_raw_data_path_ += "/calib_raw_data/cam_laser/";

    td::FileManager::InitDirectory(calib_raw_data_path_,"calib_raw_data");
    std::string bag_path  = calib_raw_data_path_ + "image_scan.bag";
    if(!bag_.isOpen())
        bag_.open(bag_path, rosbag::bagmode::Write);
    topic_name_ = "camera/rgb/image_raw";
    tdros::TFListener tf_laser_cam(nh_,"front_laser","camera_rgb_optical_frame");
    Eigen::Matrix4d T_laser_cam = Eigen::Matrix4d::Identity();
    while(!tf_laser_cam.LookupData(T_laser_cam));
    Eigen::Matrix3d Rlc(T_laser_cam.block(0,0,3,3));
    Eigen::Vector3d tlc(T_laser_cam.block(0,3,3,1));
    EulerAngles rpy =  ToEulerAngles(Eigen::Quaterniond(Rlc));

//    std::ofstream T_laser_cam_file;
    std::string T_laser_cam_file_path = calib_raw_data_path_ + "truth_pose.txt";
    cv::FileStorage T_laser_cam_file;
    td::FileManager::CreateFileCVYaml(T_laser_cam_file,T_laser_cam_file_path);
    td::FileManager::Eigen2CVYaml(T_laser_cam_file,T_laser_cam,"T_laser_cam");
    td::FileManager::Eigen2CVYaml(T_laser_cam_file,Eigen::Vector3d(rpy.roll,rpy.pitch,rpy.yaw),"RollPitchYaw");
    td::FileManager::Eigen2CVYaml(T_laser_cam_file,tlc,"txtytz");
    T_laser_cam_file.release();
}
void Calibration::SetMultiplePoseofCalibrBoard(){

    std::vector<double> translate_euler = {4.54,-2.02,0.5, 3.14,-1.5708,3.14};
//    Sophus::SE3d pose =  td::EulerTranslatetoSE3(translate_euler);
    Eigen::Vector3d eulerZ1 = {-1.57,-1.09,1.57};
    Eigen::Vector3d eulerZ3 = {1.567,-0.977,-1.57};
    Eigen::Vector3d eulerY1 = {0.0,-0.67,0.0};
    Eigen::Vector3d eulerY3 = {3.14,-1.16,3.14};
    Eigen::Vector3d eulerX1 = {-0.05,-1.5708,0.88};
    Eigen::Vector3d eulerX3 = {0.02,-1.5708,-0.62};
    std::vector<std::vector<double>> min_max(6,std::vector<double>(2));

    min_max[0][0] = init_pos_(0);
    min_max[0][1] = init_pos_(0) + 1;
    min_max[1][0] = init_pos_(1) - 0.86;
    min_max[1][1] = init_pos_(1) + 0.94;
    if(mode_ == "intrinsic"){
        min_max[2][0] = 0.85;
        min_max[2][1] = 1.34;
    } else{
        min_max[2][0] = 0.4;
        min_max[2][1] = 0.6;
    }

    //z is fixed
    min_max[3] = EulerRange(0,eulerZ1, init_euler_, eulerZ3);
    min_max[4] = EulerRange(1,eulerY1, init_euler_, eulerY3);
    min_max[5] = EulerRange(2,eulerX1, init_euler_, eulerX3);

    std::cout<<"Random choose"<<std::endl;

    Eigen::Matrix3d R2 = td::EulerToRotation(init_euler_);
    for(int i=0;i<100;i++){
        std::vector<double> translate_euler_n = translate_euler;
        for (int j = 0; j < 2; ++j) {
            translate_euler_n[j] = td::UniformSampling(min_max[j][0],min_max[j][1]);
        }
        if(mode_ == "intrinsic")
            translate_euler_n[2] = td::UniformSampling(min_max[2][0], min_max[2][1]);

        Eigen::Vector3d _euler2n = Eigen::Vector3d::Zero();
        for (int k = 0; k < 3; ++k) {
            _euler2n[k] = td::UniformSampling(min_max[k+3][0],min_max[k+3][1]);
        }
//        _euler2n[2] = td::UniformSampling(min_max[5][0],min_max[5][1]);
        Eigen::Matrix3d _R2n = td::EulerToRotation(_euler2n);
        Eigen::Matrix3d _Rn = R2 * _R2n;

        Eigen::Vector3d tn(translate_euler_n[0],translate_euler_n[1],translate_euler_n[2]);
        Sophus::SE3d posen(_Rn,tn);
        tdros::SetPosition(pose_pub_,calibr_board_name_,posen,60,80);
        recordBag(nh_,topic_name_,bag_);
        ros::Duration(1).sleep();

    }
}

void Calibration::PutCalibinInitPose() {
    Sophus::SE3d init_pose = td::EulerTranslatetoSE3(init_euler_,init_pos_);
    tdros::SetPosition(pose_pub_,calibr_board_name_,init_pose,60,80);
}

void Calibration::recordBag(ros::NodeHandle &nh, const std::string& topic, rosbag::Bag& bag) {
//    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber sub_image = it.subscribe("/head_camera/rgb/image_raw", 1, boost::bind(imageCallback,_1,std::ref(bag)));//you must hold on the sub object until you want to unsubscribe.
    bool flag_image = true;
    bool flag_scan = true;
    ros::Subscriber sub_image = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, boost::bind(imageCallback,_1,flag_image));//you must hold on the sub object until you want to unsubscribe.
    ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::LaserScan>("/front/scan", 1, boost::bind(scanCallback,_1,flag_scan));//you must hold on the sub object until you want to unsubscribe.
    flag_image = false;
    flag_scan = false;
    int i=0;
    ros::Rate loop_rate(10);
    while(ros::ok() && i<5)
    {
        ros::spinOnce();
        i++;
        loop_rate.sleep();
    }
}

void Calibration::imageCallback(const sensor_msgs::ImageConstPtr& msg, bool& flag)
{
    if(flag){
        ROS_INFO("receive image at %f", msg->header.stamp.toSec());
        bag_.write("/camera/rgb/image_raw",msg->header.stamp,*msg);
        flag = false;
    }

}
void Calibration::scanCallback(const sensor_msgs::LaserScanConstPtr& msg, bool& flag)
{
    if(flag && mode_ == "cam_laser"){
        ROS_INFO("receive scan at %f", msg->header.stamp.toSec());
        bag_.write("/front/scan",msg->header.stamp,*msg);
        flag = false;
    }

}



std::vector<double> Calibration::EulerRange(int i, Eigen::Vector3d &euler1, Eigen::Vector3d &euler2, Eigen::Vector3d &euler3) {
    Eigen::Matrix3d R1 = td::EulerToRotation(euler1);
    Eigen::Matrix3d R2 = td::EulerToRotation(euler2);
    Eigen::Matrix3d R3 = td::EulerToRotation(euler3);
    Eigen::Matrix3d R21 = R2.transpose() * R1;
    Eigen::Matrix3d R23 = R2.transpose() * R3;
    Eigen::Vector3d euler21 = td::RotationToEulerAngle(R21);
    Eigen::Vector3d euler23 = td::RotationToEulerAngle(R23);
    std::cout<<"euler21 = "<<euler21.transpose()<<"\n"<<"euler23 = "<<euler23.transpose()<<std::endl;
    std::vector<double> min_max(2);

    min_max[0] = euler21[i];
    min_max[1] = euler23[i];
    if(euler21[i] > euler23[i] )
        std::swap(min_max[0],min_max[1]);
    return min_max;
}
