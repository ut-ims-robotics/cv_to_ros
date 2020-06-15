# cv\_to\_ros
Requires opencv, [cv\_bridge](https://wiki.ros.org/vision_opencv "vision_opencv"), and [usb\_cam](http://wiki.ros.org/usb_cam "usb_cam")

`sudo apt-get install libopencv-dev`

`sudo apt-get install ros-melodic-usb-cam`

`sudo apt-get install ros-melodic-vision-opencv`

build workspace and launch usb\_cam

`roslaunch usb_cam usb_cam-test.launch`

and after that

`rosrun cv_to_ros usb_cam_cv.py`

## Running everything with custom parameters
1) Simply run:

  ```bash
  roslaunch cv_to_ros usb_cam_cv.launch
  ```

2) You can make current parameters persistent by entering:

  ```bash
  roscd cv_to_ros
  rosparam dump config/detection_params.yaml /cv_to_ros
  ```
