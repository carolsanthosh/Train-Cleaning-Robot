# Camera Streamer
ROS package to stream camera image to a web interface with object detection on RPi-2

## Summary
This package uses three rospy pakcages.

-camera_publisher.py : gets camera data and publishes it to **/usb_camera/image/compresses** rostopic 

-camera_streamer.py :  subscribes to **usb_camera/image/compresses** and **/bounding_box** rostopics and streams the output image to web interface

-object_detector.py : subscribes to **usb_camera/image/compresses** to perform object detection using tflite runtime library and publishes the detected bounding boxes to  **/bounding_box**


    
## How to run

### Get ROS Package
ROS workspace

    cd ~/catkin_ws/src
if you do not have a workspace, create one.

    mkdir -p /catkin_ws/src
    cd ~/catkin_ws/src
You can change the workspace name as you desire

Git Clone


    git clone https://github.com/carolsanthosh/train-cleaning-robot.git

catkin make

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    
After this try running the ROS package using roslaunch command to have all three python files

    roslaunch camera_streamer streamer.launch

Now you should see the web interface ip address displayed on the terminal 
or
search in browser,

    http://localhost:5000
Depending on the device you use, change the url to **http://ipaddress:5000**. the **ipaddress** should be the host ip address of the machine running the rospackage 
to view it in any devices connected in the same network


### Changing parameters

    roscd camera_streamer/launch

In streamer.launch file, you could change the below parameters if required.

    <arg name="lbl_path" default="$(find camera_streamer)/models/labelmap.txt" />
    <arg name="model_path" default="$(find camera_streamer)/models/detect_quant.tflite" />
    <arg name="camera_index" default="1" />

