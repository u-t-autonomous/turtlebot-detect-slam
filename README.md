# turtlebot-detect-slam

This repository contains documentation and resources for doing visual SLAM (simultaneous localization and mapping) on a [TurtleBot3 Waffle Pi](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) and ROS. This repository is a work in progress and will be constantly changing as I research and experiment with different packages and tools. 

# Setup

### Turtlebot Setup

- Follow the setup guide in the [ROBOTIS e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup).

    - Everything in this repository is tested using Ubuntu 20.04 with ROS Noetic on the Remote PC, and the provided Raspbian disk image in the manual ([Section 6.2.1.2](https://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#install-linux-based-on-raspbian)).
    - The Remote PC you use should have a somewhat powerful Nvidia GPU if you want the segmentation/detection packages to run at a respectable framerate. 

### Object Detection Setup - YOLOv3

- Follow installation instructions for the [darknet ROS package](https://github.com/leggedrobotics/darknet_ros).
    - I found [this script](https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh) helpful for installing OpenCV.
    - [CUDA installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html). Make sure you follow the mandatory post-installation steps!

### Optional Setup

- I recommend installing [ROS web video server](http://wiki.ros.org/web_video_server) so you can easily look at different image topics in your web browser. Otherwise you can use `rqt_image_view` or something.

```sudo apt-get install ros-melodic-web-video-server```

# Usage

- Start up ROS master. SSH into the turtlebot to start up its sensors and its camera with the following commands.

```roslaunch turtlebot3_bringup turtlebot3_robot.launch```

```roslaunch raspicam_node camerav2_1280x960.launch```

- Run the launch file (on the Remote PC).

```roslaunch visual_slam classify_and_locate.launch```

- To use [custom weights](https://github.com/leggedrobotics/darknet_ros#use-your-own-detection-objects), either edit `yolo_v3.launch` in the `darknet_ros` package or make a new launch file that points to the proper weights/configuration files.

## Simulation

This package also has support for Gazebo simulation, with the [Turtlebot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) package as a prerequisite. The launch file opens an empty world by default.

```roslaunch visual_slam gazebo_classify_and_locate.launch```

# Weights files

bevo.weights - Bevo figurine only
turtlebot.weights - Turtlebot_burger only
multiclass.weights
    - box
    - bevo
    - bottle
    - footwear
    - turtlebot
