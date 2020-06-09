# cv_to_ros
Requires opencv, [cv_bridge](https://wiki.ros.org/vision_opencv "vision_opencv"), and [usb_cam](http://wiki.ros.org/usb_cam "usb_cam")

`sudo apt-get install libopencv-dev`

`sudo apt-get install ros-melodic-usb-cam`

`sudo apt-get install ros-melodic-vision-opencv`

build workspace and luanch usb_cam

`roslaunch usb_cam usb_cam-test.launch`

and after that

`rosrun cv_to_ros usb_cam_cv_py.py`