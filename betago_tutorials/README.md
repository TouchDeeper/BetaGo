# Tutorial Package for BetaGo 
This is tutorial package for BetaGo.
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
