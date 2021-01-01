# D435i Calibration
## SDK installation
| Required Info                         |                                                                |
|---------------------------------|------------------------------------------- |
| Camera Model                       | D400 | 
| Firmware Version                   | 05.12.09.00 | 
| Operating System & Version |   Ubuntu 16 | 
| ros Version |   kinetic | 
| Kernel Version (Linux Only)    |  4.15.0-128-generic             | 
| Platform                                 | PC |
| SDK Version                            |  2.40.0                      | 
| Language                            | C                         | 
| Segment			|  Robot                  | 
| installation Method			|  backend installation refer to https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md                  | 
## IMU
### Intrinsic calibration
refert to [IMU_calibration_tool](https://dev.intelrealsense.com/docs/imu-calibration-tool-for-intel-realsense-depth-camera)

the `rs-imu-calibration` is in the [librealsense root]//tools/rs-imu-calibration

### Allan analysis
- matlab installation. Refer to [here](https://www.jianshu.com/p/3db9122e5bec)
    - choose the `license_standalone.lic` instead of the `license_server.lic`.
    - install `matlab-support`, `sudo apt-get install matlab-support`
    - start matlab, `sudo /usr/local/MATALB/R2018a/bin/matlab`
        - if error `Version `GLIBCXX_3.4.22' not found` occur, refer to [solution](https://stackoverflow.com/a/46613765)
    - make start up icon. `sudo gedit /usr/share/applications/matlab.desktop`, add the content to the gedit and modify to your case.
        ```
        [Desktop Entry]
        Type=Application
        Name=Matlab
        GenericName=MATLAB
        Comment=Matlab:The Language of Technical Computing
        Exec=sh /home/td/MATLAB/R2018a/bin/matlab -desktop
        Icon=/home/td/MATLAB/R2018a/toolbox/shared/dastudio/resources/MatlabIcon.png
        StartupNotify=true
        Terminal=false
        Categories=Development;Matlab;
        ```
     - In the end, `sudo rm -rf .matlab`.

- record the imu bag exceeds 2 hours.
    1. move the `rs_d435i_calibration.launch` in this directory to the realsense_ws and run it to start the realsense d435i. The mainly modification is to set `<arg name="unite_imu_method"          default="linear_interpolation"/>`.
    2. ` rosbag record -O imu_calibration /camera/imu`
#### imu_utils
1. imu_utils installation: 
    1. clone the code_utils to the workspace and `catkin_make`. 
        - problem
            1. `fatal error: backward.hpp: No such file or directory`.
        - solution
            1. refer to [here](https://github.com/gaowenliang/imu_utils/issues/12#issuecomment-473818718)        
    2. clone the imu_utils to the workspace and `catkin_make`.

3. Allan analysis by `imu_utils`
    1. move the `d435i_imu_calibration.launch` in this directory  to the imu_utils_ws and run it.
    2. `rosbag play -r 100 imu_calibration.bag`. `-r 100` is to play the bag by 100 times speed.
    3. the analysis result will be save in the imu_utils/data. The related file will named \*d435i*.*
    4. The `gyr_n` of result is discrete `gyroscope_noise_density`,  you can refer to [here](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) for the convertion between continue-time `gyroscope_noise_density` and discrete `gyroscope_noise_density`. `acc_n` is the same as `gyr_n`.
 #### kalibr_allan
 1. clone the kalibr_allan to the workspace and `catkin_make`  
 2. if Eigen is not found
    - if eigen is not installed:
        install eigen
    - if eigen has been installed: 
        1. replace `find_package(Eigen3 REQUIRED)` to `include_directories("/usr/local/include/eigen3" )` in the CMakeLists.txt
        2. delete the Eigen in `catkin_package` in the CMakeLists.txt
 3. if Matlab is not found
    - change the `MATLAB_EXE_PATH` in `bagconvert//FindMatlab.cmake` like below:
        ```
          find_program(MATLAB_EXE_PATH matlab
              PATHS /usr/local/MATLAB/R2018a/bin)
        ```
    - delete the `build` and `devel` and `catkin_make`.
 4. `rosrun bagconvert bagconvert [your.bag] [imu_topic_name]` to convert the bag to mat
 5. modify the `mat_path` and `update_rate` to your case in SCRIPT_allan_matparallel.m, then run it.
 
 ## Camera
 ### <a id="multi-camera">  Intrinsic and extrinsic of infra1, infra2 and color camera joint calibration using Kalibr </a>
 
 1. Install Kalibr
 2. prepare calibration target.
    1. Download the pdf and yaml of `Aprilgrid 6x6 0.5x0.5 m (unscaled)` in [here](https://github.com/ethz-asl/kalibr/wiki/downloads).
    2. Crop the aprilgrid from the pdf to png, and then put the aprilgrid to visio. Scale the image to make the tag size is an integer centimeter number. You should measure the tag size and space, then modify the yaml file according to [here](https://github.com/ethz-asl/kalibr/wiki/calibration-targets#a-aprilgrid). Screen ruler can measure the size in screen, baidu pan link:
        ```
       https://pan.baidu.com/s/1YQN8Rf820Em23VCjoZ5ABQ
       password: aqif 
       ```
    3. turn off the laser emitter and auto exposure by realsense-viewer, and then save the json file. You will need to add this json file path the launch file of realsense.
    4. less the screen's brightness until there are no bright spot in infra1, infra2, color camera.
 3. copy the `rs_kalibr_multi_camera.launch` in this directory to realsense_ws, and run it. There are something need to note:
     1. if the fps of camera is too low(in my case. 4HZ), the calibration will occur error `Cameras are not connected through mutual observations`, I guess the low fps makes the time difference of synced image topics larger and exceed the threshold of Kalibr. So I increase the fps to 10HZ, the error gone. 
 4.  record bag by `rosbag record -O multi_camera.bag /infra_left /infra_right /color`.
 5. use Kalibr to calibrate:
    ```
    kalibr_calibrate_cameras --targe ../kalibr_target/april_6x6_50x50cm.yaml --bag multi_camera.bag --models pinhole-radtan pinhole-radtan pinhole-radtan --topics /infra_left /infra_right /color
    ```
 6. result yaml file Interpretation. 
    1. the `T_cn_cnm1` always take itself as base frame, the previous camera is the child frame
        ```
        cam0:
            ......
        cam1:
            T_cn_cnm1:
        cam2:
            T_cn_cnm1:
        ```
 ## infra1, infra2, imu joint calibration
 1. copy the `rs_imu_camera_kalibr.launch` in this directory to realsense_ws,
 2. `roslaunch realsense2_camera rs_imu_camera_kalibr.launch`
    - you can check the fps of `/infra1_left`, `/infra_right` and `/IMU` by `rostopic hz /infra_left /infra_right /imu`, then modify the fps argument in the below node in `rs_imu_camera_kalibr.launch` to make the image fps approximately 
    ```
      <node name="throttle_infra1" pkg="topic_tools" type="throttle" args="messages /camera/infra1/image_rect_raw 30.0 /infra_left" />
      <node name="throttle_infra2" pkg="topic_tools" type="throttle" args="messages /camera/infra2/image_rect_raw 30.0 /infra_right" />
      <node name="throttle_imu" pkg="topic_tools" type="throttle" args="messages /camera/imu 262 /imu" />
    ```
    - If timestamp of IMU have two kinds of base number, sometime the secs of timestamp is like 1608015601, sometime the secs of timestamp is like 3216031523, you can refer to this [issuse](https://github.com/IntelRealSense/realsense-ros/issues/1569) for detail. A quick fix is set the `initial_rest` to true in `rs_imu_camera_kalibr.launch`
 2. record bag, `rosbag record -O imu_infra_left.bag /infra_left /infra_right /imu`
 3. calibration
    ```
    kalibr_calibrate_imu_camera --bag imu_infra1_infra2.bag --cam camera-imu-yaml/camchain-multi_camera1.yaml --imu camera-imu-yaml/imu.yaml --target ../kalibr_target/april_6x6_50x50cm.yaml
    ```
    - the `camchain-multi_camera1.yaml` is the result of [multi-camera-calibration](#multi-camera). Here we just calibrate the `/infra1_left`, `/infra_right` and `/IMU`, so we need to delete the color camera related content.
    - `imu.yaml` is the result of `imu_utils`, but you need to write the `imu.yaml` by yourself according to this [tutorial](https://github.com/ethz-asl/kalibr/wiki/yaml-formats#imu-configuration-imuyaml)
 ## Modification in Kalibr
 - Error like `ImportError: cannot import name NavigationToolbar2Wx ` occurs, a quick fix is [here](https://github.com/ethz-asl/kalibr/issues/202#issuecomment-403417656)
 - Error like `Spline Coefficient Buffer Exceeded. Set larger buffer margins!` occurs, a quick fix is [here](https://github.com/ethz-asl/kalibr/issues/41#issuecomment-179706154), I set the `timeOffsetPadding` to 0.1.