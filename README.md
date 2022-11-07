# Usage-introduction
## ROS Installation
### Step1 add ROS software source

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Step2 add key
```shell
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Step3 Installation ROS
``` shell
sudo apt update
sudo apt install ros-melodic-desktop-full
```

The different version of ubuntu should install different ros.The name would be different.

### Step4 Init rosdep and update rosdep

``` shell
sudo rosdep init
rosdep update
```

### Step5 add ros environment variable

```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Step6 test code

```shell
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```

## MAVROS Installation



## vrpn_client_ros Installation

When install vrpn_client_ros, The name between ros and vrpn would be different. You should choose the version of your ros 

```shell
sudo apt-get install ros-kinetic-vrpn-client-ros
```

## Optitrack Usage

### Step1 Open optitrack and mavros

```shell
roslaunch vrpn_client_ros sample server:=[optitrack ip]
roslaunch mavros px4.launch
```

### Step2 launch your own code

