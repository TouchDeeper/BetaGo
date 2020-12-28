# Navigation Package for BetaGo 
This is navigation package for BetaGo.
## Dependencies
- Gmapping: `sudo apt-get install ros-kinetic-gmapping*`
- Cartographer: `sudo apt-get install ros-kinetic-cartographer*`
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
![kinect + lidar + odom mapping result](../media/rtabmap_3.png)

### VINS-Fusion
1. calibrate the realsense d435i first according to `betago_calibration/doc/rs_d435i_calibration.md`
2. copy the `rs_vins_fusion.launch` to realsense_ws and `roslaunch realsense2_camera rs_vins_fusion.launch `
3. copy the yaml file in `config/vins-fusion` to `vins_fusion_ws/VINS-Fsuion/realsense_d443i/`
4. in vins_ws:
    ```
   roslaunch vins vins_rviz.launch
   rosrun vins vins_node /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_me.yaml
   ```
## File explanation
None
## Notes
None

## Modify on other project used in BetaGo
None
