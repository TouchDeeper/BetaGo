# D435i Calibration
## IMU
### Intrinsic calibration
refert to [IMU_calibration_tool](https://dev.intelrealsense.com/docs/imu-calibration-tool-for-intel-realsense-depth-camera)

the `rs-imu-calibration` is in the [librealsense root]//tools/rs-imu-calibration

### Allan analysis
- matlab installation. Refer to [csdn](https://blog.csdn.net/hitzijiyingcai/article/details/81989031)
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
    - set the `MATLAB_ROOT` to your installation path, in my case, add `set(MATLAB_ROOT /home/td/MATLAB/R2016b)` in the CMakeLists.txt.
    - delete the `build` and `devel` and `catkin_make`.
 4. `rosrun bagconvert bagconvert [your.bag] [imu_topic_name]` to convert the bag to mat
 5. modify the `mat_path` and `update_rate` to your case in SCRIPT_allan_matparallel.m, then run it.
 
 ## Camera
 ### Intrinsic and extrinsic of infra1, infra2 and color camera joint calibration using Kalibr
 
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
 ## Camera-imu joint calibration
 1. 
 2. record bag, `rosbag record -O imu_infra_left.bag /infra_left /imu`
 3. calibration
    ```
    kalibr_calibrate_imu_camera --bag imu_infra_left.bag --cam camchain-multi_camera1.yaml --imu imu.yaml --target ../kalibr_target/april_6x6_50x50cm.yaml
    ```