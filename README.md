# coListener

## Project Introduction

coListener is a universal ROS topic listener that can subscribe to specified ROS topics and process data. It supports both ROS1 and ROS2, automatically adapting to different ROS environments.

## Features

* Dual ROS version support: Compatible with both ROS1 and ROS2 environments
* Multi-platform support: Supports amd64, arm64, armhf and other hardware platforms
* Multiple ROS distributions: Supports ROS1's indigo, ROS2's foxy and humble
* Persistent storage: Uses SQLite database to store message data
* Custom actions: Supports executing custom actions for received messages
* Batch processing: Periodically sends and processes collected messages in batches
* Supported platforms

| ROS distro | platform     | Ubuntu distro |
|------------|--------------|---------------|
| noetic     | amd64, arm64 | focal         |
| foxy       | amd64, arm64 | focal         |
| humble     | amd64, arm64 | jammy         |
| indigo     | armhf        | trusty        |
| melodic    | amd64, arm64 | bionic        |

## Installation

Install using apt

  ```bash
    curl -fsSL https://apt.coscene.cn/coscene.gpg | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/coscene.gpg
    echo "deb [signed-by=/etc/apt/trusted.gpg.d/coscene.gpg] https://apt.coscene.cn $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/coscene.list
    sudo apt update
    sudo apt install ros-$ROS_DISTRO-colistener
  ```

## Download deb for installation

| Platform | ROS Distro | Ubuntu Distro | URL                                                                                           |
|----------|------------|---------------|-----------------------------------------------------------------------------------------------|
| amd64    | noetic     | focal         | https://apt.coscene.cn/dists/focal/main/binary-amd64/ros-noetic-colistener_latest_amd64.deb   |
| arm64    | noetic     | focal         | https://apt.coscene.cn/dists/focal/main/binary-arm64/ros-noetic-colistener_latest_arm64.deb   |
| amd64    | melodic    | bionic        | https://apt.coscene.cn/dists/bionic/main/binary-amd64/ros-melodic-colistener_latest_amd64.deb |
| arm64    | melodic    | bionic        | https://apt.coscene.cn/dists/bionic/main/binary-arm64/ros-melodic-colistener_latest_arm64.deb |
| amd64    | foxy       | focal         | https://apt.coscene.cn/dists/focal/main/binary-amd64/ros-foxy-colistener_latest_amd64.deb     |
| arm64    | foxy       | focal         | https://apt.coscene.cn/dists/focal/main/binary-arm64/ros-foxy-colistener_latest_arm64.deb     |
| amd64    | humble     | jammy         | https://apt.coscene.cn/dists/jammy/main/binary-amd64/ros-humble-colistener_latest_amd64.deb   |
| arm64    | humble     | jammy         | https://apt.coscene.cn/dists/jammy/main/binary-arm64/ros-humble-colistener_latest_arm64.deb   |
| armhf    | indigo     | trusty        | https://apt.coscene.cn/dists/trusty/main/binary-armhf/ros-indigo-colistener_latest_armhf.deb  |

## Build from source (Recommended)

  ```
    # ROS1
    cd ~/catkin_ws/src
    git clone https://github.com/coscene-io/coListener.git
    cd ..
    catkin_make install
    source install/setup.bash
    
    # ROS2
    cd ~/ros2_ws/src
    git clone https://github.com/coscene-io/coListener.git
    cd ..
    colcon build
    source install/setup.bash
  ```

## Usage

* ROS1
  ```
  # Source ROS environment variables first (e.g.: source /opt/ros/noetic/setup.bash)
  roslaunch colistener colistener.launch
  ```

* ROS2
  ```
  # Source ROS environment variables first (e.g.: source /opt/ros/humble/setup.bash)
  ros2 launch colistener colistener.launch.xml
  ```
  
* For rule engine related content, see: [Add Rule](https://docs.coscene.cn/docs/use-case/data-diagnosis/add-rule)

### Configuration Parameters

**ROS1 configuration file path: /opt/ros/$ROS_DISTRO/share/colistener/launch/colistener.launch**

**ROS2 configuration file path: /opt/ros/$ROS_DISTRO/share/colistener/launch/colistener.launch.xml**

| Parameter Name           | Data Type     | Default Value                                                                  | Description                        |
|--------------------------|---------------|--------------------------------------------------------------------------------|------------------------------------|
| subscribe_topics         | String Array  | ['/error_report', '/event_tracking_report']                                   | List of topics to subscribe to     |
| persistence_expire_secs  | Integer       | 1800                                                                          | Message data expiration time (sec) |
| persistence_file_path    | String        | "/tmp/colistener/persistence/ros1.db" or "/tmp/colistener/persistence/ros2.db" | Persistence database file path     |
| action_type              | String        | "common"                                                                      | Action type                        |
| log_directory            | String        | "/tmp/colistener/logs/"                                                       | Log storage directory              |

**❗After modifying the configuration file, you need to restart coListener**

## Building Release Packages

This project uses GitHub Actions to automatically build and publish Debian packages. When a new tag is pushed or a new release is created, the workflow automatically updates the version number and builds the corresponding Debian package.

## Development Guide

* Adding a new action type
    1. Create a new action class header file in the listener_base/include/actions/ directory
    2. Implement the action class in the listener_base/src/actions/ directory
    3. Add registration and identification of the action type in the main program

## License

This project is licensed under the Apache License 2.0. See the LICENSE file for details.

## Contact

Maintainer: coscene@coscene.io
