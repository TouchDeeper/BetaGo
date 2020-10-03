//
// Created by wang on 20-7-25.
//

#include "betago_calibration/Calibration.h"
void Calibration::SetMultiplePoseofCalibrBoard(){

    std::vector<double> translate_euler = {1.54,-0.02,0.5, 3.14,-1.5708,3.14};
    Sophus::SE3d pose =  td::EulerTranslatetoSE3(translate_euler);
    Eigen::Vector3d eulerZ1 = {-1.57,-1.09,1.57};
    Eigen::Vector3d eulerZ3 = {1.567,-0.977,-1.57};
    Eigen::Vector3d eulerY1 = {0.0,-0.67,0.0};
    Eigen::Vector3d eulerY3 = {3.14,-1.16,3.14};
    Eigen::Vector3d eulerX1 = {-0.05,-1.5708,0.88};
    Eigen::Vector3d eulerX3 = {0.02,-1.5708,-0.62};
    std::vector<std::vector<double>> min_max(6,std::vector<double>(2));

    min_max[0][0] = 1.54;
    min_max[0][1] = 2.53;
    min_max[1][0] = -0.88;
    min_max[1][1] = 0.92;
    //z is fixed
    min_max[3] = EulerRange(0,eulerZ1, init_euler_, eulerZ3);
    min_max[4] = EulerRange(1,eulerY1, init_euler_, eulerY3);
    min_max[5] = EulerRange(2,eulerX1, init_euler_, eulerX3);

    std::cout<<"Random choose"<<std::endl;

    Eigen::Matrix3d R2 = td::EulerToRotation(init_euler_);
    for(int i=0;i<10;i++){
        std::vector<double> translate_euler_n = translate_euler;
        for (int j = 0; j < 2; ++j) {
            translate_euler[j] = td::UniformSampling(min_max[j][0],min_max[j][1]);
        }
        Eigen::Vector3d _euler2n;
        for (int k = 0; k < 3; ++k) {
            _euler2n[k] = td::UniformSampling(min_max[k+3][0],min_max[k+3][1]);
        }
        Eigen::Matrix3d _R2n = td::EulerToRotation(_euler2n);
        Eigen::Matrix3d _Rn = R2 * _R2n;
        Eigen::Vector3d _eulern = td::RotationToEulerAngle(_Rn);

        translate_euler_n[3] = _eulern[0];
        translate_euler_n[4] = _eulern[1];
        translate_euler_n[5] = _eulern[2];

        Sophus::SE3d posen =  td::EulerTranslatetoSE3(translate_euler_n);
        tdros::SetPosition(pose_pub_,calibr_board_name_,posen,60,80);
        recordBag(nh_,topic_name_,bag_);
        ros::Duration(1).sleep();

    }
}

void Calibration::PutCalibinInitPose() {
    Sophus::SE3d init_pose = td::EulerTranslatetoSE3(init_euler_,init_pos_);
    tdros::SetPosition(pose_pub_,calibr_board_name_,init_pose,60,80);
}

void Calibration::recordBag(const ros::NodeHandle &nh, const std::string& topic, rosbag::Bag& bag) {
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("camera/rgb/image_raw", 1, boost::bind(imageCallback,_1,std::ref(bag)));//you must hold on the sub object until you want to unsubscribe.
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, rosbag::Bag& bag)
{
    bag.write("/camera/rgb/image_raw",ros::Time::now(),msg);
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
