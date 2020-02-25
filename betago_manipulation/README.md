# Manipulation Package for BetaGo 
This is manipulation packages for BetaGo.
## Usage
### Only Rviz
```asm
roslaunch betago_moveit_config demo.launch
```
The node graph:
![only rviz node graph](https://github.com/TouchDeeper/BetaGo/blob/ztd/media/demo_graph.png)

### With Gazebo
```asm
roslaunch betago_bringup betago_bringup_moveit.launch
roslaunch betago_moveit_config moveit_rviz.launch
```
args of `betago_bringup_moveit.launch`:
- `dual_arm:=(default:true/false)` : single-arm controller or dual-arm controller
## Step to construct the package for moveit+gazebo
1. use moveit setup_assistant to construct the initial package.
2. copy the betago_moveit_planning_executation.launch to betago_moveit_config/launch and change the content to suit your model.
3. copy the controllers.yaml to betago_moveit_config/config and change the content to suit your model.
4. copy the content in the launch/ridgeback_moveit_controller_manager.launch.xml to the same file in your package.


## File explanation
None
## Notes
None
## Modify on other project used in BetaGo
None