# Turtlebot3-ROS
University of Edinburgh SDP 2022 Gourp15 Turtlebot3 ROS Codebase
#### This repo and readme are currently managed and monitored by Ivan Zhong, pin me if readme doesn't work.
#### Last update: 18:06 6, March

![IMG_0837](https://user-images.githubusercontent.com/16554153/156164482-addfe0e1-b855-40ec-a639-04a393f34f06.jpeg)


Package Name: painted

Turtlebot type: Burger

Quick start guide is based on DICE machine and I assume your already got ros installed.
If you running vm on your own machine, follow the setup guide [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) first, then skip step 2.


## Quick start guide
#### 1.Initial catkin_ws (skip this step if you already done so)

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
I suggest you add setup.bash into your bashrc file.

open~/.bashrc and at the end of the file, add this:

```
source ~/catkin_ws/devel/setup.bash
```

#### 2.git clone dependencies (skip this if you done setup guide [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup))
```
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
(TODO add more here for no sudo machine)
cd ~/catkin_ws
catkin_make
```

#### 3.git clone this repo into your ~/catkin_ws/src folder
```
cd ~/catkin_ws/src
git clone https://github.com/SDP2022/Turtlebot3-ROS.git
cd ~/catkin_ws
catkin_make
```

#### 4.run simulation
```
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
roslaunch src/painted/launch/simulation.launch
```
Now you should see gazebo running and painted software started.

## Running painted code on robot
SSH into robot and launch painted software:
```
ssh -XC pi@tentacool
(password: turtlebot)
cd ~/catkin_ws
roslaunch src/painted/launch/painted.launch
```

## Repo structure

```
.
├── CMakeLists.txt
├── launch
│   ├── painted.launch  # Launch file for initial turtlebot and starting painted software.
│   ├── simulation.launch # Launch file starting simulation and painted software
│   └── turtlebot.launch # Launch file for initial turtlebot hardware only
├── package.xml
├── README.md
├── scripts # Test files for testing services, subject to remove but feel free to learn how to service work from it
│   ├── add_two_ints_client.py
│   └── add_two_ints_server.py
├── src
│   ├── control.py # Control node(completed)
│   ├── execute_demo.py # Execute service client demo
│   ├── master.py # Master node [WIP]
│   ├── move.py # demo1 movement file
│   ├── move_test.py # demo1 movement file
│   ├── pen.py # Pen node (completed)
│   └── slam_data_json_output.py # Script for outputing /map data into json file
└── srv # Folder for all service messages
    ├── AddTwoInts.srv # Test file, subject to remove
    ├── ControlCommand.srv # Control node service msg define
    └── PenCommand.srv # Pen node service msg define
```
## Services info
#### ControlCommand.srv
Execute control command. Note that could only execute either displacement or rotation at every call. 
Set value to 0 for no displacement or rotation.

Input:
float64 displacement #displacement in meter
float64 rotation #rotation in degree, positive clockwise negative anticlockwise

Output:
bool status #execution status, should always return true

#### PenCommand.srv
Execute pen command. Note that if pen is already down and call pen_down=True again it won't move motor and return false as status.

Input:
bool pen_down #True for moving pen down, False for moving pen up

Output:
bool status #execution status. True if motor does move. False if doesn't


## Useful command and QA
1. Uploading code into turtlebot
  ```
scp -r ~/catkin_ws/src/painted pi@tentacool:~/catkin_ws/src
  ```

2. Error: Can't import painted.srv

  - try source setup file again
  
  ```
source ~/catkin_ws/devel/setup.bash
  ```
