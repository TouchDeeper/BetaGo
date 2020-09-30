# Calibration Package for BetaGo 
This is Calibration package for BetaGo.

## Calibration simulation system
A Calibration simulation system to play with the Calibration in Gazebo
### CamLaserCalibration
kalibr_tag: apriltag grid gazebo model, refer to [kalibr](https://github.com/ethz-asl/kalibr/wiki/Calibration-targets#a-aprilgrid) for detail of this kind of Calibration target.
put this file in to `~/.gazebo/models/`, then you can load the kalibr_tag directly in the Gazebo.

## Usage
```asm
roslaunch betago_calibration cam_laser_calib_world.launch
roslaunch betago_calibration rviz.launch
```