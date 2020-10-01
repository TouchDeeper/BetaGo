# Calibration Package for BetaGo 
This is Calibration package for BetaGo.

## Calibration simulation system
A Calibration simulation system to play with the Calibration in Gazebo
### CamLaserCalibration
- Make sure you have download the gazebo models and set the GAZEBO_MODEL_PATH environment variable as [this](../README.md 'Simulation') said.
- kalibr_tag: apriltag grid gazebo model, refer to [kalibr](https://github.com/ethz-asl/kalibr/wiki/Calibration-targets#a-aprilgrid) for detail of this kind of Calibration target.
put this file in to `~/.gazebo/models/`, then you can load the kalibr_tag directly in the Gazebo.

```asm
roslaunch betago_calibration cam_laser_calib_world.launch
roslaunch betago_calibration rviz.launch
```