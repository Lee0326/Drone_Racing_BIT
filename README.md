# Drone_Racing_BIT

## Getting Started

The code was developed in Ubuntu 18.04, ROS Melodic and gazebo9.  PX4 is installed only for software in loop simulation. 

### Install ROS

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt-get install ros-melodic-desktop
# Source ROS
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
rosdep init
rosdep update
```

### Install Dependencies

Follow [mavros_installation](https://dev.px4.io/en/ros/mavros_installation.html) to install mavros:

```
#install gazebo9
sudo apt install ros-melodic-gazebo9*
#install mavros
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

### Install the drone race package

```
git clone https://github.com/Lee0326/Drone_Racing_BIT.git
cd Drone_Racing_BIT
git submodule init && git submodule update
catkin_make
```

### Install PX4 SITL  

According to the instruction [ROS with Gazebo Simulation](https://dev.px4.io/master/en/simulation/ros_interface.html) and [Development Environment on Ubuntu](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html) to build the PX4

```
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/ubuntu.sh
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/requirements.txt
source ubuntu.sh
## optional, if you only plan to use px4 to do simulation
INSTALL_NUTTX="false"
git clone https://github.com/PX4/Firmware
cd Firmware
git submodule update --init --recursive
git checkout v1.11.0-beta1
make distclean
make px4_sitl_default gazebo
```

#### Source the PX4 environment :

Open the bashrc file:

```
sudo gedit ~/.bashrc 
```

Add the following command to the end of the file:

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${your Drone_Racing_BIT path}/Drone_Racing_BIT/src/simulation/gazebo_simulation/models
source ${your px4 path}/Firmware_v110/Tools/setup_gazebo.bash ${your px4 path}/Firmware_v110 ${your px4 path}/Firmware_v110/build/px4_sitl_default
source ${your Drone_Racing_BIT path}/Drone_Racing_BIT/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware_v110
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware_v110/Tools/sitl_gazebo
```

## Let's Race!

#### Test the indoor race track in gazebo

First, start the gazebo simulation along with px4 sitl and mavros:

```
roslaunch drone_racing_simulation indoor_race_track.launch
```

When the drone is armed and takeoff to the desired initial position, then:

```
rosrun minimum_snap_trajectory minimum_snap_trajectory_node
```

The drone follows the globally planned trajectory containing all the gates position as waypoints (1.5xspeed) :

![indoor](files/indoor.gif)

#### Run the simulation in rviz:

```
roslaunch so3_quadrotor_simulator simulator.launch
rosrun gate_visualization gate_visualization
```

![indoor](files/rviz.gif)