# Calibration Package for BetaGo 
This is Calibration package for BetaGo.

## Calibration simulation system
A Calibration simulation system to play with the Calibration in Gazebo

## Usage

1. Make sure you have download the gazebo models and set the GAZEBO_MODEL_PATH environment variable as [this](../README.md 'Simulation') said.
2. kalibr_tag: apriltag grid gazebo model, refer to [kalibr](https://github.com/ethz-asl/kalibr/wiki/Calibration-targets#a-aprilgrid) for detail of this kind of Calibration target.
put this file in to `~/.gazebo/models/`, then you can load the kalibr_tag directly in the Gazebo. Refer to the [Calibration_targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets#a-aprilgrid)
 for how to generate a kalibr_tag.
 

### Intrinsic calibration

1. Record bag include image and laser scan data.

    ```asm
    roslaunch betago_calibration cam_laser_calib_world.launch
    roslaunch betago_calibration rviz.launch
    rosrun betago_calibration cam_laser_calibration_node _mode:=intrinsic
    ```
2. Calibrate camera intrinsic. Refer to [kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration#2-running-the-calibration). An example:

    ```
    kalibr_calibrate_cameras --bag ../BetaGo/BetaGo_ws/src/betago_calibration/calib_raw_data/intrinsic/image_scan.bag --topics /camera/rgb/image_raw --models pinhole-radtan --target ../BetaGo/BetaGo_ws/src/betago_calibration/calibration_target/kalibr_tag/april_6x6_80x80cm.yaml
    ```

    If error about initialization of focal length occur, set the initial value of focal length by:
    
    `export KALIBR_MANUAL_FOCAL_LENGTH_INIT=[init_guess]`
    
    then run the previous command again. After that, you need write down the initial value in the terminal then the calibration program can go on.
3. Calibration validator

    In Kalibr workspace, open a terminal, then
    ```asm
    source devel/setup.bash
    kalibr_camera_vidator --cam camchain-..BetaGoBetaGo_wssrcbetago_calibrationtest.yaml --target ../BetaGo/BetaGo_ws/src/betago_calibration/kalibr_tag/april_6x6_80x80cm.yaml
    ```
    
### CamLaserCalibration


1. Record bag include image and laser scan data.

    ```asm
    roslaunch betago_calibration cam_laser_calib_world.launch
    roslaunch betago_calibration rviz.launch
    rosrun betago_calibration cam_laser_calibration_node
    ```
    
2. copy the intrinsic calibration result to the config file of CamLaserCalibraTool.

3. Compute pose of calibration target

    `roslaunch lasercamcal_ros kalibra_apriltag.launch `

4. Cam-laser calibration

    `roslaunch lasercamcal_ros calibra_offline.launch`
    
### Hand-Eye Calibration

