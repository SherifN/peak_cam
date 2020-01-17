# peak_cam

A Linux ROS C++ Node that wraps the driver API for IDS vision cameras using IDS peak software. Tested on Ubuntu 18.04 LTS.

## How to run

### Before running the code

1. install [ROS](http://wiki.ros.org/ROS/Installation)
2. install [IDS peak](https://de.ids-imaging.com/download-vision-lin64.html)

### Running the code

0. Clone the repository to your Linux computer

1. Generate a ROS workspace

    `mkdir -p camera_ws/src/` 

1. Copy the peak_cam package into your ROS workspace and build it
    
    `cp -r peak_cam/ camera_ws/src/`
    
    `cd camera_ws/ && catkin_make && source devel/setup.bash`

2. Set parameters such as ROS-Topic and acquisition rate in `launch/params/peak_cam_params.yaml`

3. Plug the IDS vision camera and launch the node 

    `roslaunch peak_cam peak_cam_node.launch`


> Hint: Sometimes the cameras are only accesible as root. Try `sudo -s` in your terminal and launch the node again.


Copyright (c) 2020, Sherif Nekkah

All rights reserved.

BSD license: see LICENSE file
