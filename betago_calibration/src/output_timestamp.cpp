//
// Created by wang on 20-12-8.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <td_ros/subscriber/imu_subscriber.hpp>
#include <TdLibrary/slam_tool/output.hpp>
#include <TdLibrary/FileOperation/file_manager.hpp>
#include <td_ros/sensor_data/pose_data.hpp>
#include "sophus/se3.hpp"
#include <fstream>
using namespace tdros;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "output_timestamp");
    ros::NodeHandle nh;
    td::FileManager file_manager;
    std::ofstream ofs_timestamp;
    ofs_timestamp.setf(std::ios_base::fixed, std::ios_base::floatfield);
    ofs_timestamp.precision(10);
    std::string imu_timestamp_path = ros::package::getPath("betago_calibration")+"/calib_raw_data";
    std::string file_name = "imu_timestamp2.txt";
    file_manager.CreateDirectory(imu_timestamp_path);
    std::string full_path = imu_timestamp_path + "/" + file_name;
    file_manager.CreateFile(ofs_timestamp, full_path, 4);

    std::shared_ptr<tdros::IMUSubscriber> imu_sub_ptr = std::make_shared<tdros::IMUSubscriber>(nh, "/imu/data", 100000);

    std::deque<tdros::IMUData> imu_data_buff;
    int output_index =  0;
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        imu_sub_ptr->ParseData(imu_data_buff);

        while (!imu_data_buff.empty() ) {
            tdros::IMUData imu_data = imu_data_buff.front();
            double t = imu_data.time;
            ofs_timestamp<<t<<std::endl;
            imu_data_buff.pop_front();

        }
        rate.sleep();
    }
    ofs_timestamp.close();
    return 0;
}
