# Clion ROS Development Note
## Find package that in another workspace
The key is to set the `ROS_PACKAGE_PATH`
- method 1:

    `source [path_to_workspace]/devel/setup.bash --extend`
    
- methdo 2(clion):

    open `File->settings->environment`, find the `ROS_PACKAGE_PATH` and add `[path_to_workspace]/src`
    