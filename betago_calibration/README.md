# Calibration Package for BetaGo 
This is Calibration package for BetaGo.

## Calibration simulation system
A Calibration simulation system to play with the Calibration in Gazebo
### CamLaserCalibration
1. Make sure you have download the gazebo models and set the GAZEBO_MODEL_PATH environment variable as [this](../README.md 'Simulation') said.
2. kalibr_tag: apriltag grid gazebo model, refer to [kalibr](https://github.com/ethz-asl/kalibr/wiki/Calibration-targets#a-aprilgrid) for detail of this kind of Calibration target.
put this file in to `~/.gazebo/models/`, then you can load the kalibr_tag directly in the Gazebo. Refer to the [Calibration_targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets#a-aprilgrid)
 for how to generate a kalibr_tag.

3. Record bag include image and laser scan data.

    ```asm
    roslaunch betago_calibration cam_laser_calib_world.launch
    roslaunch betago_calibration rviz.launch
    rosrun betago_calibration cam_laser_calibration_node
    ```

4. Calibratie camera intrinsic. Refer to [kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration#2-running-the-calibration). An example:

    ```
    kalibr_calibrate_cameras --bag ../BetaGo/BetaGo_ws/src/betago_calibration/test.bag --topics /camera/rgb/image_raw --models pinhole-radtan --target ../BetaGo/BetaGo_ws/src/betago_calibration/kalibr_tag/april_6x6_80x80cm.yaml
    ```

    If error about initialization of focal length occur, set the initial value of focal length by:
    
    `export KALIBR_MANUAL_FOCAL_LENGTH_INIT=[init_guess]`
    
    then run the previous command again. After that, you need write down the initial value in the terminal then the calibration program can go on.
