# Overview
This repository is about the BetaGo robot maintained by South China University of Technology [PEMT Laboratory](http://www.scut.edu.cn/pemt/).

![BetaGo](media/BetaGo.gif)

![rtabmap](media/rtabmap.gif)

![BetaGo](media/moveit+gazebo.gif)

## Dependencies
- moveit-ros packages:
```
sudo apt-get install ros-kinetic-moveit-ros-*
sudo apt-get install ros-kinetic-moveit
```
- ridgeback packages:
```
sudo apt-get install ros-kinetic-ridgeback*
```
- UR5 packages:
```
sudo apt-get install ros-kinetic-ur-*
sudo apt-get install ros-kinetic-ur5*
```

- [TdRosLib](https://github.com/TouchDeeper/TdRosLib)

Clone the TdRosLib to your path, and modify the path in [this line](https://github.com/TouchDeeper/BetaGo/blob/d622e6610427a00c21e55e944fb27b1148e0630c/betago_manipulation/betago_bringup/CMakeLists.txt#L32) of CMakeLists.txt.

## Usage
create the workspace

`mkdir ~/BetaGo_ws && cd ~/BetaGo_ws`

clone this repositories and build

```
git clone https://github.com/TouchDeeper/BetaGo.git src/
catkin_make
```

### Simulation
- ur5 and allegro_hand are imported by environment variable RIDGEBACK_URDF_EXTRAS. RIDGEBACK_URDF_EXTRAS = your workspace path/src/betago_description/urdf/betago/ridgeback_urdf_extras.xacro
     - method1: set temporary environment variable:`export RIDGEBACK_URDF_EXTRAS=~/BetaGo_ws/src/betago_description/urdf/betago/ridgeback_urdf_extras.xacro`
    - method2: set permanent environment variable:
        1. terminal:`sudo gedit ~/.bashrc`
        2. add `export RIDGEBACK_URDF_EXTRAS=~/BetaGo_ws/src/betago_description/urdf/betago/ridgeback_urdf_extras.xacro` in the end.
        3. terminal:`source ~/.bashrc`
        
- Check if the model has been downloaded in `~/.Gazebo/`, if not, you need to download the model first and put it in `~/.Gazebo/`. [Download link](https://bitbucket.org/osrf/gazebo_models/downloads/).
For Chinese, download from [rosclub.cn](http://www.rosclub.cn/post-37.html) : link: http://pan.baidu.com/s/1pKaeg0F, password:cmxc

### Real robot
#### Step to connect to the ridgeback by wired connection
1. plug in the network cable.
2. start the ridgeback, then push the E-STOP to release the lock. 
3. set the wired connection in your desktop, see the [gif](https://github.com/TouchDeeper/BetaGo/blob/ztd/media/set_wire_network.gif).
4. add the config in the below to your ~/.bashrc, and `source ~/.bashrc`
```
export ROS_MASTER_URI=http://192.168.131.1:11311 #Ethernet
export ROS_IP=192.168.131.100
```
5.add the config `192.168.131.1 CPR-R100-0057`in the below to your /etc/hosts, the location is after the line like 
```
127.0.0.1	localhost
127.0.1.1	[your-computer-name]
```
6.`rostopic list` in your desktop to see if the topics in the ridgeback have been send to your desktop. If receive the topic, the connection is valid.
### step to login the ridgeback's pc by ssh
1. ssh administrator@192.168.131.1
### Step to run the package
- ur5 and allegro_hand are imported by environment variable RIDGEBACK_URDF_EXTRAS. RIDGEBACK_URDF_EXTRAS = your workspace path/src/betago_description/urdf/betago/ridgeback_urdf_extras.xacro
     - method1: set temporary environment variable:`export RIDGEBACK_URDF_EXTRAS=your workspace path/src/betago_description/urdf/betago/ridgeback_urdf_extras.xacro`
    - method2: set permanent environment variable:
        1. terminal:`sudo gedit ~/.bashrc`
        2. add `export RIDGEBACK_URDF_EXTRAS="your workspace path/src/betago_description/urdf/betago/ridgeback_urdf_extras.xacro"` in the end.
        3. terminal:`source ~/.bashrc`
   
## File explanation
None
## Notes
None
