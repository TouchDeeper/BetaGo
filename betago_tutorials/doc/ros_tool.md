# ROS tool Note

### extract image from image topic
- dependencies
    ```$xslt
    $ sudo apt-get install mjepgtools
    $ sudo apt-get install ffmpeg
    ```
 - record
 ```$xslt
rosrun image_view extract_images _sec_per_frame:=0.01 image:=<IMAGETOPICINBAGFILE> # <IMAGETOPICINBAGFILE> is the image topic you want to save
```