# rqt Tutorial for BetaGo 
## Dependencies
- `pip install matplotlib`
- 
```
  sudo apt-get install ros-kinetic-rqt
  sudo apt-get install ros-kinetic-rqt-common-plugins
  ```
## rqt_bag
`rosrun rqt_bag rqt_bag`
record or open a  bag.
- Troubleshooting 
    - Unindexed bag: `rosbag reindex [your_bag]`

## [rqt_plot](http://wiki.ros.org/rqt_plot)

note:
 1. the format of the value, for example, we want to view the angular velocity along Z axis of `/ridgeback_velocity_controller/odom`, 
    - `rostopic info /ridgeback_velocity_controller/odom `, it is a type of `nav_msgs/Odometry`
    - `rosmsg show nav_msgs/Odometry`, the output is below, so the topic you need type in rqt_plot_gui is `/ridgeback_velocity_controller/odom/twist/twist/angular/z`
        ```
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        string child_frame_id
        geometry_msgs/PoseWithCovariance pose
          geometry_msgs/Pose pose
            geometry_msgs/Point position
              float64 x
              float64 y
              float64 z
            geometry_msgs/Quaternion orientation
              float64 x
              float64 y
              float64 z
              float64 w
          float64[36] covariance
        geometry_msgs/TwistWithCovariance twist
          geometry_msgs/Twist twist
            geometry_msgs/Vector3 linear
              float64 x
              float64 y
              float64 z
            geometry_msgs/Vector3 angular
              float64 x
              float64 y
              float64 z
          float64[36] covariance
        ```
   2. zoom out. in the `zoom to rectangle` tool, the left click in zoom in, and the right click is zoom out.
## Moveit!
### move_group_interface_tutorial
This section is refer to the  [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html) of the Moveit!
```asm
roslaunch betago_moveit_config demo.launch betago_tutorial:=true
roslaunch betago_tutorials move_group_interface_tutorial.launch
```
Note: If there is no **RvizVisualToolsGui** in your rviz panel, you can add the configuration code below in the betago_tutorial.rviz manually
```asm
- Class: rviz_visual_tools/RvizVisualToolsGui
    Name: RvizVisualToolsGui
```

### MoveIt! Commander Scripting
This section is refer to the  [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/moveit_commander_scripting/moveit_commander_scripting_tutorial.html) of the Moveit!
```asm
roslaunch betago_moveit_config demo.launch betago_tutorial:=true
rosrun moveit_commander moveit_commander_cmdline.py
```

### The RobotModel and RobotState Classes
This section is refer to the  [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html) of the Moveit!
```asm
roslaunch betago_tutorials robot_model_and_robot_state_tutorial.launch
```
### Planning Scene
This section is refer to the  [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html) of the Moveit!
```asm
roslaunch betago_tutorials planning_scene_tutorial.launch
```

### Planning Scene ROS API
This section is refer to the  [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html) of the Moveit!
```asm
roslaunch betago_moveit_config demo.launch betago_tutorial:=true
roslaunch betago_tutorials planning_scene_ros_api_tutorial.launch
```

### Motion Planning API
This section is refer to the  [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/motion_planning_api/motion_planning_api_tutorial.html) of the Moveit!
```asm
roslaunch betago_moveit_config demo.launch betago_tutorial:=true
roslaunch betago_tutorials motion_planning_api_tutorial.launch 
```

### Motion Planning Pipeline
This section is refer to the  [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/motion_planning_pipeline/motion_planning_pipeline_tutorial.html) of the Moveit!
```asm
roslaunch betago_tutorials motion_planning_pipeline_tutorial.launch

```

### BetaGo tf listener tutorial
This section is refer to the  [tutorial](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29) of the tf!
```asm
rosrun betago_tutorials betago_tf_listener
```