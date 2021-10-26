# peak_cam

A Linux [ROS C++ Node](https://wiki.ros.org/peak_cam) that wraps the driver API for IDS vision cameras using IDS peak software. Tested on Ubuntu 18.04 LTS and 20.04 LTS.

## How to run

### Before running the code

1. Install [ROS](http://wiki.ros.org/ROS/Installation)
1. Install [IDS peak](https://de.ids-imaging.com/download-vision-lin64.html)
    1. Install dependencies: `sudo apt install libqt5core5a libqt5gui5 libqt5widgets5 libqt5quick5 qml-module-qtquick-window2 qml-module-qtquick2 qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qtquick-layouts qml-module-qt-labs-settings qml-module-qt-labs-folderlistmodel libusb-1.0-0`
    1. Install package: `sudo dpkg -i ids-peak-<version>-<arch>.deb`
    1. (Fix OpenCV version issue if working on NVIDIA Jetson: `sudo ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv`)
1. Test connection to your camera using `ids_visioncockpit`
1. (Update camera firmware if needed: `ids_deviceupdate -s *<last-four-digits-serialnumber> -U --guf <path-to-guf-file>`)

#### Configuration for GigE cameras
1. Enable jumboframes for the ethernet interface(s) (e.g., `eth0`) used for your camera(s): `ip link set dev eth0 mtu 9000`
1. Increase receive buffer size [as recommended in the IDS manual](https://en.ids-imaging.com/manuals/ids-peak/ids-peak-user-manual/1.3.1/en/operate-gige-hints-linux.html): `sudo /usr/local/scripts/ids_set_receive_buffer_size.sh`
1. Make sure to adjust `DeviceLinkThroughputLimit` in the `.yaml` configuration file according to your desired framerate and available hardware (employing separate network interfaces may be beneficial)

### Running the code

1. Clone the repository to your Linux computer

1. Generate a ROS workspace

    `$ mkdir -p camera_ws/src/` 

1. Copy the peak_cam package into your ROS workspace and build it
    
    `$ cp -r peak_cam/ camera_ws/src/`
    
    `$ cd camera_ws/ && catkin_make && source devel/setup.bash`

1. Set parameters such as ROS topic and acquisition rate under `launch/params/peak_cam_params.yaml`

1. Plug the IDS vision camera and launch the node 

    `$ roslaunch peak_cam peak_cam_node.launch`
    
1. Stop the node with `Ctrl-C` (SIGINT) for controlled shutdown 

For multiple cameras, create a `.launch` and a `.yaml` file for each camera.

> Hint: Sometimes the cameras are only accesible as root. Try ` sudo -s` in your terminal and launch the node again.

### Triggering for stereo vision
To synchronously acquire images from two cameras in a stereo vision setup, you might want to let one camera trigger the other. 

1. Setup the GPIO wiring [as proposed by IDS](https://en.ids-imaging.com/application-notes-details/app-note-synchronizing-image-acquisition.html) according to the pin layout found in your camera's datasheet (example color coding for GV-5270FA):

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

### Trigger using external pulses
The cameras can also be triggered by the pulses of external devices such as a lidar sensor.

1. (Activate the trigger output of your external device, e.g., using the `multipurpose_io_mode` flag for Ouster lidars as discussed in [section 5.2.2 of their software user manual](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2.1.0.pdf#35))
1. Setup the GPIO wiring in compliance to your hardware, e.g., with an optoisolated open collector sync pulse circuit:
    
    ![External trigger wiring](docs/wiring-external-trigger.png)

1. Configure your camera's `.yaml` files with
    ```
    Line1Source: "Off"
    TriggerSource: "Line0"
    ```

Copyright (c) 2020, Sherif Nekkah, Felix Keppler (Fraunhofer IVI) and Contributors 

All rights reserved.

BSD license: see LICENSE file
