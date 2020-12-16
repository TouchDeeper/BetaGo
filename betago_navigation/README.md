# Navigation Package for BetaGo 
This is navigation package for BetaGo.
## Simulation
### Gmapping
- read the navigation [tutorial](http://www.clearpathrobotics.com/assets/guides/ridgeback/navigation.html) of the ridgeback
- change `roslaunch ridgeback_gazebo ridgeback_world.launch` to `roslaunch betago_bringup betago_bringup_moveit.launch`
- The rest is the same as the tutorial said.
### rtabmap
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
1. calibrate the realsense d435i first according to `betago_calibration/doc/rs_d435i_calibration.launch`
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
