//
// Created by wang on 20-12-8.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <td_ros/subscriber/odometry_subscriber.hpp>
#include <td_ros/publisher/odometry_publisher.hpp>
#include <TdLibrary/slam_tool/output.hpp>
#include <TdLibrary/FileOperation/file_manager.hpp>
#include <sophus/se3.hpp>

using namespace tdros;

bool moveEnough(Eigen::Matrix4f& last, Eigen::Matrix4f& now, float th ){
    Sophus::SE3f Tlast(last);
    Sophus::SE3f Tnow(now);
    Sophus::SE3f Tlast_now = Tlast.inverse() * Tnow;
    td::Vec6f se3_last_now = Tlast_now.log();
    if(se3_last_now.norm()>th)
        return true;
    return false;
//    std::cout<<<<std::endl;
}
int main(int argc, char *argv[]) {
//    google::InitGoogleLogging(argv[0]);
//    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
//    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "odometry_output_node");
    ros::NodeHandle nh;
    td::FileManager file_manager;
    std::ofstream ofs_groundtruth;
    std::ofstream ofs_estimate;
    std::string groundtruth_path = ros::package::getPath("betago_navigation")+"/output/path/groundtruth.txt";
    std::string estimate_path = ros::package::getPath("betago_navigation")+"/output/path/estimate.txt";
    file_manager.CreateFile(ofs_groundtruth,groundtruth_path,4);
    file_manager.CreateFile(ofs_estimate,estimate_path,4);

    std::shared_ptr<tdros::OdometrySubscriber> ground_truth_sub_ptr = std::make_shared<tdros::OdometrySubscriber>(nh, "/ground_truth/state", 100000);
    std::shared_ptr<tdros::OdometrySubscriber> estimate_sub_ptr = std::make_shared<tdros::OdometrySubscriber>(nh, "/odometry/filtered", 100000);

    std::shared_ptr<tdros::OdometryPublisher> groundtruth_map_pub_ptr = std::make_shared<tdros::OdometryPublisher>(nh, "/groundtruth/map/state", "/map", "/groundtruth_odom_map", 100);
    std::shared_ptr<tdros::OdometryPublisher> keyframe_pub_ptr = std::make_shared<tdros::OdometryPublisher>(nh, "/odometry/keyframe", "/map", "/keyframe", 100);

    std::deque<tdros::PoseData> groundtruth_data_buff;
    std::deque<tdros::PoseData> estimate_data_buff;
//    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
//    bool transform_received = false;
//    bool gnss_origin_position_inited = false;
    int output_index =  0;
    Eigen::Matrix4f last_keyframe;
    bool last_keyframe_init = false;
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        ground_truth_sub_ptr->ParseData(groundtruth_data_buff);
        estimate_sub_ptr->ParseData(estimate_data_buff);

//        if (!transform_received) {
//            if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
//                transform_received = true;
//                // LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
//            }
//        } else {
        while (!groundtruth_data_buff.empty()  && !estimate_data_buff.empty()) {
            tdros::PoseData groundtruth_data = groundtruth_data_buff.front();
            tdros::PoseData estimate_data = estimate_data_buff.front();
            Eigen::Matrix4f odometry_matrix;
            odometry_matrix = groundtruth_data.pose;
            odometry_matrix(0,3) -= 3;

            groundtruth_map_pub_ptr->Publish(odometry_matrix);

            double d_time = estimate_data.time - groundtruth_data.time;
            if (d_time < -0.05) {
                estimate_data_buff.pop_front();
            } else if (d_time > 0.05) {
                groundtruth_data_buff.pop_front();
            } else {
                if(!last_keyframe_init)
                {
                    last_keyframe = estimate_data.pose;
                    last_keyframe_init = true;
                }else{
                    if(moveEnough(last_keyframe,estimate_data.pose,0.5))
                    {
                        keyframe_pub_ptr->Publish(odometry_matrix);
                        td::saveTrajectoryTUM(ofs_estimate,estimate_data.pose,output_index);
                        td::saveTrajectoryTUM(ofs_groundtruth, groundtruth_data.pose, output_index);
                        last_keyframe = estimate_data.pose;
                        output_index++;
                    }
                }
                estimate_data_buff.pop_front();
                groundtruth_data_buff.pop_front();

            }
        }
//        }

        rate.sleep();
    }

    return 0;
}
