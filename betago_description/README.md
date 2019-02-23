# Description Package for BetaGo 
This is description packages for BetaGo.

## File explanation
This section is used to explain what some files do.
- launch/view_betago_rviz.launch: view betago in rviz.

## Notes


## Modify on other project used in BetaGo
### ridgeback
urdf/ridgeback/ridgeback.gazebo:
```
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
          <robotNamespace>/</robotNamespace>
    </plugin>
```
change to
```
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
          <robotNamespace>/</robotNamespace>
          <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
          <legacyModeNS>true</legacyModeNS>
    </plugin>
```
or it will occur some error as the [link](https://answers.ros.org/question/292444/gazebo_ros_control-plugin-gazeboroscontrolplugin-missing-legacymodens-defaultrobothwsim/) said.
