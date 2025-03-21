# coListener

## 项目介绍

coListener 是一个通用的 ROS 话题监听器，可以订阅指定的 ROS 话题并进行数据处理。它同时支持 ROS1 和 ROS2，能够自动适应不同的
ROS 环境。

## 特点

* 双 ROS 版本支持：兼容 ROS1 和 ROS2 环境
* 多平台支持：支持 amd64, arm64, armhf 等多种硬件平台
* 多 ROS 发行版：支持 ROS1 的 indigo，ROS2 的 foxy 和 humble
* 持久化存储：使用 SQLite 数据库存储消息数据
* 自定义动作：支持针对接收到的消息执行自定义动作
* 批量处理：定时批量发送和处理收集到的消息
* 支持的平台

| ROS distro | platform     | Ubuntu distro |
|------------|--------------|---------------|
| noetic     | amd64, arm64 | focal         |
| foxy       | amd64, arm64 | focal         |
| humble     | amd64, arm64 | jammy         |
| indigo     | armhf        | trusty        |
| melodic    | amd64, arm64 | bionic        |

## 安装

使用 apt 安装

  ```bash
    curl -fsSL https://download.coscene.cn/coscene-apt-source/coscene.gpg | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/coscene.gpg
    echo "deb [signed-by=/etc/apt/trusted.gpg.d/coscene.gpg] https://download.coscene.cn/coscene-apt-source $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/coscene.list
    sudo apt update
    sudo apt install ros-$ROS_DISTRO-colistener
  ```

## 下载deb安装

| Platform | ROS Distro | Ubuntu Distro | URL                                                                                                                                             |
|----------|------------|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| amd64    | noetic     | focal         | https://download.coscene.cn/coscene-apt-source/dists/focal/main/binary-amd64/ros-noetic-colistener_latest_amd64.deb   |
| arm64    | noetic     | focal         | https://download.coscene.cn/coscene-apt-source/dists/focal/main/binary-arm64/ros-noetic-colistener_latest_arm64.deb   |
| amd64    | melodic    | bionic        | https://download.coscene.cn/coscene-apt-source/dists/bionic/main/binary-amd64/ros-melodic-colistener_latest_amd64.deb |
| arm64    | melodic    | bionic        | https://download.coscene.cn/coscene-apt-source/dists/bionic/main/binary-arm64/ros-melodic-colistener_latest_arm64.deb |
| amd64    | foxy       | focal         | https://download.coscene.cn/coscene-apt-source/dists/focal/main/binary-amd64/ros-foxy-colistener_latest_amd64.deb     |
| arm64    | foxy       | focal         | https://download.coscene.cn/coscene-apt-source/dists/focal/main/binary-arm64/ros-foxy-colistener_latest_arm64.deb     |
| amd64    | humble     | jammy         | https://download.coscene.cn/coscene-apt-source/dists/jammy/main/binary-amd64/ros-humble-colistener_latest_amd64.deb   |
| arm64    | humble     | jammy         | https://download.coscene.cn/coscene-apt-source/dists/jammy/main/binary-arm64/ros-humble-colistener_latest_arm64.deb   |
| armhf    | indigo     | trusty        | https://download.coscene.cn/coscene-apt-source/dists/trusty/main/binary-armhf/ros-indigo-colistener_latest_armhf.deb  |

## 从源码构建 (推荐)

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

## 使用方法

* ROS1
  ```
  # 使用前需要先source ROS 环境变量（e.g.: source /opt/ros/noetic/setup.bash）
  roslaunch colistener colistener.launch
  ```

* ROS2
  ```
  # 使用前需要先source ROS 环境变量（e.g.: source /opt/ros/humble/setup.bash）
  ros2 launch colistener colistener.launch.xml
  ```
  
* 规则引擎相关内容见： [添加规则](https://docs.coscene.cn/docs/use-case/data-diagnosis/add-rule)

### 配置参数

**ROS1 配置文件路径： /opt/ros/$ROS_DISTRO/share/colistener/launch/colistener.launch**

**ROS2 配置文件路径： /opt/ros/$ROS_DISTRO/share/colistener/launch/colistener.launch.xml**

| 参数名                     | 数据类型  | 默认值                                                                           | 描述          |
|-------------------------|-------|-------------------------------------------------------------------------------|-------------|
| subscribe_topics        | 字符串数组 | ['/error_report', '/event_tracking_report']                                   | 要订阅的话题列表    |
| persistence_expire_secs | 整数    | 1800                                                                          | 消息数据过期时间(秒) |
| persistence_file_path   | 字符串   | "/tmp/colistener/persistence/ros1.db" 或 "/tmp/colistener/persistence/ros2.db" | 持久化数据库文件路径  |
| action_type             | 字符串   | "common"                                                                      | 动作类型        |
| log_directory           | 字符串   | "/tmp/colistener/logs/"                                                       | 日志存储目录      |

**❗修改配置文件后，需要重启coListener**

## 构建发布包

此项目使用 GitHub Actions 自动构建和发布 Debian 包。当推送新的标签或创建新的发布时，工作流会自动更新版本号并构建对应的
Debian 包。

## 开发指南

* 添加新的动作类型
    1. 在 listener_base/include/actions/ 目录下创建新的动作类头文件
    2. 在 listener_base/src/actions/ 目录下实现该动作类
    3. 在主程序中添加动作类型的注册和识别

## 许可证

本项目基于 Apache License 2.0 授权。详情请参阅 LICENSE 文件。

## 联系方式

维护者: coscene@coscene.io