# turtlebot-visual-slam

This repository contains documentation and resources for doing visual SLAM (simultaneous localization and mapping) on a [TurtleBot3 Waffle Pi](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) and ROS. This repository is a work in progress and will be constantly changing as I research and experiment with different packages and tools. 

# Setup

### Turtlebot Setup

- Follow the setup guide in the [ROBOTIS e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup).

    - Everything in this repository is tested using Ubuntu 18.04 with ROS Melodic on the Remote PC (I know the guide says to use Kinetic), and the provided Raspbian disk image in the manual ([Section 6.2.1.2](https://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#install-linux-based-on-raspbian)).
    - The Remote PC you use should have a somewhat powerful Nvidia GPU if you want the segmentation/detection packages to run at a respectable framerate. 

### YOLO v3 Setup

- Follow installation instructions for the [darknet ROS package](https://github.com/leggedrobotics/darknet_ros).
    - I found [this script](https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh) helpful for installing OpenCV.
    - I had some issues with OpenCV 4, switching to OpenCV 3.4 fixed those issues.
    - [CUDA installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html). Make sure you follow the mandatory post-installation steps!

### Semantic Segmentation Setup

TODO

### Optional Setup

- I recommend installing [ROS web video server](http://wiki.ros.org/web_video_server) so you can easily look at different image topics in your web browser. Otherwise you can use `rqt_image_view` or something.

```sudo apt-get install ros-melodic-web-video-server```

# Usage

I will probably add some ROS launch files to this repository that automate everything once I've finalized what packages will be used. 

- Start up ROS master. SSH into the turtlebot to start up its sensors and its camera with the following commands.

```roslaunch turtlebot3_bringup turtlebot3_robot.launch```

```roslaunch raspicam_node camerav2_1280x960.launch```

- All of the image processing nodes need messages of type `sensor_msgs/Image`, but enabling the raw footage from the Raspberry Pi's camera has very high latency and low framerate. By default, the node publishes a compressed image (`sensor_msgs/CompressedImage`). To use this compressed image, we need to republish the image as a `sensor_msgs/Image`.

```rosrun image_transport republish compressed in:=/raspicam_node/image out:=/camera/decompressed```

- You can  name the `out` topic to whatever you want, just make sure all of your image processing packages are subscribed to that topic, not `raspicam_node/image`!

- To make sure your republishing worked, use ROS web video server or `rqt_image_view` to view your `out` topic and you should see the live camera feed from the Turtlebot. 

### YOLO

- Edit the parameters in `darknet_ros/config/ros.yaml` so that the node is subscribed to the correct topic. 

- Launch the ROS node.

```roslaunch darknet_ros yolov3.launch```

### Semantic Segmentation

To-do
