# Navigation Package for BetaGo 
This is navigation package for BetaGo.
## Simulation
### Gmapping
- read the navigation [tutorial](http://www.clearpathrobotics.com/assets/guides/ridgeback/navigation.html) of the ridgeback
- change `roslaunch ridgeback_gazebo ridgeback_world.launch` to `roslaunch betago_gazebo betago_world.launch`
- The rest is the same as the tutorial said.
### rtabmap
- only kinect
```
 roslaunch betago_gazebo betago_world.launch
 roslaunch betago_navigation rtabmap_mapping_kinect.launch simulation:=true
 roslaunch betago_navigation rtabmap_rviz.launch
```
- kinect + lidar
``` 
 roslaunch betago_gazebo betago_world.launch
 roslaunch betago_navigation rtabmap_mapping_kinect_scan.launch simulation:=true
 roslaunch betago_navigation rtabmap_rviz.launch
```

## File explanation
None
## Notes
None

## Modify on other project used in BetaGo
None
