# peak_cam

A Linux [ROS C++ Node](https://wiki.ros.org/peak_cam) that wraps the driver API for IDS vision cameras using IDS peak software. Tested on Ubuntu 18.04 LTS and 20.04 LTS.

## How to run

### Before running the code

1. install [ROS](http://wiki.ros.org/ROS/Installation)
2. install [IDS peak](https://de.ids-imaging.com/download-vision-lin64.html)

### Running the code

0. Clone the repository to your Linux computer

1. Generate a ROS workspace

    `$ mkdir -p camera_ws/src/` 

1. Copy the peak_cam package into your ROS workspace and build it
    
    `$ cp -r peak_cam/ camera_ws/src/`
    
    `$ cd camera_ws/ && catkin_make && source devel/setup.bash`

2. Set parameters such as ROS topic and acquisition rate under `launch/params/peak_cam_params.yaml`

3. Plug the IDS vision camera and launch the node 

    `$ roslaunch peak_cam peak_cam_node.launch`
    
4. Stop the node with `Ctrl-C` (SIGINT) for controlled shutdown 

For multiple cameras, create a `.launch` and a `.yaml` file for each camera.

> Hint: Sometimes the cameras are only accesible as root. Try ` sudo -s` in your terminal and launch the node again.

### Triggering and stereo vision
To synchronously acquire images from two cameras in a stereo vision setup, you might want to let one camera trigger the other. 

1. Setup the GPIO wiring [as proposed by IDS](https://en.ids-imaging.com/application-notes-details/app-note-synchronizing-image-acquisition.html) according to the pin layout found in your camera's datasheet:
![Flash output trigger wiring](docs/wiring-flash-output-trigger-input.png)

1. Configure the primary (master) camera's `.yaml` file with
    ```
    Line1Source: "ExposureActive"
    TriggerSource: "Off"
    ```
    and the secondary camera's `.yaml` with
    ```
    Line1Source: "Off"
    TriggerSource: "Line0"
    ```

Copyright (c) 2020, Sherif Nekkah and Contributors 

All rights reserved.

BSD license: see LICENSE file
