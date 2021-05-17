# Navigation Package for BetaGo 
This is navigation package for BetaGo.
## Dependencies
- Gmapping: 
    - `sudo apt-get install ros-kinetic-gmapping*`
- Cartographer: 
    - `sudo apt-get install ros-kinetic-cartographer*`
- Rtabmap: 
    - `sudo apt-get install ros-kinetic-octomap*`
## Simulation
### Gmapping
- read the navigation [tutorial](http://www.clearpathrobotics.com/assets/guides/ridgeback/navigation.html) of the ridgeback
- change `roslaunch ridgeback_gazebo ridgeback_world.launch` to `roslaunch betago_navigation navigation_world.launch`
- The rest is the same as the tutorial said.

### Hector-SLAM
```
 roslaunch betago_navigation navigation_world.launch
 roslaunch betago_navigation hector_demo.launch
```
### Cartographer
#### Build.
- kinetic
    1. refer to [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation) until `# Only on Ubuntu 16 / ROS Kinetic: src/cartographer/scripts/install_proto3.sh`.
    2. install protoc3 in system default path will conflict with the protoc2.6.1 needed by ros, so we need to install protoc3 in custom local path. add `-DCMAKE_INSTALL_PREFIX=../install \` to `src/cartographer/scripts/install_proto3.sh`
        ```
           cmake -G Ninja \
             -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
             -DCMAKE_BUILD_TYPE=Release \
             -Dprotobuf_BUILD_TESTS=OFF \
             -DCMAKE_INSTALL_PREFIX=../install \
             ../cmake
       ```
    3. in `build and install` step, run 
        ```
        catkin_make_isolated --install --use-ninja -DCMAKE_PREFIX_PATH="${PWD}/install_isolated;${PWD}/protobuf/install;${CMAKE_PREFIX_PATH}"
       ```
    4. If you have install the protoc3 in system default path and encounter the conflict problem:
        1. `cd [protobuf3_dir]/build`
        2. remove the protobuf3, `cat install_manifest.txt | sudo xargs rm`
        3. reinstall protobuf2.6.1. refer to [here](https://blog.csdn.net/lwplwf/article/details/76532804).
- ros version after kinetic just refer to [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation).
### rtabmap
note: this is only test in `ros-kinetic-rtabmap-ros`
- only kinect
```asm
 roslaunch betago_navigation navigation_world.launch
 roslaunch betago_navigation rtabmap_sim_only_kinect.launch simulation:=true
 roslaunch betago_navigation rtabmap_rviz.launch
```
- kinect + odom
```asm
 roslaunch betago_navigation navigation_world.launch
 roslaunch betago_navigation rtabmap_sim_kinect_odom.launch simulation:=true
 roslaunch betago_navigation rtabmap_rviz.launch
```
- kinect + lidar + odom
```asm
 roslaunch betago_navigation navigation_world.launch
 roslaunch betago_navigation rtabmap_sim_kinect_scan_odom.launch simulation:=true
 roslaunch betago_navigation rtabmap_rviz.launch
```
#### Bag record
```
rosbag record -b 0 --split --size=5120 -O 3d2.bag -a -x "/wifi(.*)|/twist(.*)|/rosout(.*)|/status|/set_pose|/mcu(.*)|/diagnostics(.*)|/cmd_lights|/bluetooth(.*)|/disparity|/gps/fix|/imu_filter_node(.*)|/initialpose|/left/(.*)|/move_base_simple/goal|/rgbd_image_relay|/right/(.*)|/rtabmap(.*)|/tag_detections|/user_data_async|/voxel_cloud|(.*)/compressed(.*)|/feature_tracker(.*)|/feedback|/vins(.*)|/camera/aligned_depth_to_infra1(.*)"
```
- `-b 0`: set the bag buffer size to infinite, otherwise the bag buffer will exceed as the warning said `rosbag record buffer exceeded.  Dropping oldest queued message.`.
- `--split --size=5120`: split the bag if the size reach 5G.
![kinect + lidar + odom mapping result](../media/rtabmap_3.png)


### VINS-Fusion
In betago workspace:
```asm
 roslaunch betago_navigation navigation_world.launch
 roslaunch betago_navigation rviz_control_gazebo.launch
```
In vins-fusion workspace:
```
rosrun vins vins_node /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback_gazebo.yaml
```
## REAL
### VINS-Fusion
#### Bag record
```
rosbag record -O rtabmap1.bag -a -x "/wifi(.*)|/twist(.*)|/rosout(.*)|/status|/set_pose|/mcu(.*)|/diagnostics(.*)|/cmd_lights|/bluetooth(.*)|/disparity|/gps/fix|/imu_filter_node(.*)|/initialpose|/left/(.*)|/move_base_simple/goal|/rgbd_image_relay|/right/(.*)|/rtabmap(.*)|/tag_detections|/user_data_async|/voxel_cloud|(.*)/compressed(.*)"
```
if some error about `Compressed Depth Image Transport ` occur, it means there is some topic with `compressedDepth`,
if there is `/camera/infra1/image_rect_raw/compressedDepth/(.*)`, you need to disable the image_transport plugin in launch file by:
```
<group ns="/camera/infra1/image_rect_raw" >
   <rosparam param="disable_pub_plugins">
      - 'image_transport/compressedDepth'
      - 'image_transport/theora'
   </rosparam>
</group>
```
#### Realsense d435i
1. calibrate the realsense d435i first according to `betago_calibration/doc/rs_d435i_calibration.md`
2. copy the `rs_vins_fusion.launch` to realsense_ws and `roslaunch realsense2_camera rs_vins_fusion.launch `
3. copy the yaml file(`realsense_stereo_imu_config_me.yaml`,`infra1.yaml`,`infra2.yaml`) in `config/vins-fusion` to `vins_fusion_ws/VINS-Fsuion/realsense_d435i/`
4. in vins_ws:
    ```
   roslaunch vins vins_rviz.launch
   rosrun vins vins_node /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_me.yaml
   ```
#### Realsense d435 and ridgeback's IMU
1. get the initial value of extrinsic parameters from the simulation robot.
    1. `rosrun tf tf_echo /imu_link /D435i_camera_left_ir_optical_frame`, get the rotation and translation of `body_T_cam0`.
        output pattern:
        ```
        At time 0.000
        - Translation: [0.042, 0.308, 0.544]
        - Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
                    in RPY (radian) [-1.571, 0.000, -1.571]
                    in RPY (degree) [-90.000, 0.000, -90.000]
       ```
         The last value of `Quaternion` is the real part.
         `RPY` is fixed axis rotation in roll->pitch->yaw order, that is `ZYX` convention said below, the number order of `[-1.571, 0.000, -1.571]` is still the `(x,y,z)/(roll, pitch, yaw)` order .
    2. the quaternion can be converted to rotation matrix by this [online tool](https://www.andre-gaschler.com/rotationconverter/).
        The Euler convention of this [online tool](https://www.andre-gaschler.com/rotationconverter/) is current axis rotation. For example, the `ZYX` convention means rotate by Z axis then rotate by current Y axis then rotate by current X axis, the blank need to fill of (x,y,z) is (roll, pitch, yaw).
    3. copy the yaml file(`realsense_stereo_imu_config_ridgeback.yaml`, `infra1.yaml`, `infra2.yaml`) in `config/vins-fusion` to `vins_fusion_ws/VINS-Fsuion/realsense_d435i/`
    4. in vins_ws:
        ```
       roslaunch vins vins_rviz.launch
       rosrun vins vins_node /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
       ```
## File explanation
None
## Notes
None
