# SJTU Sailboat Project
Simulation: https://github.com/sjtu-automatic-maritime-system/Sailboat-Simulation

Lab: https://github.com/sjtu-automatic-maritime-system/Sailboat-Lab

ROS官方下载地址：http://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment

#### 1. install
```
mkdir -p ~/sailboat_ws/src
cd ~/sailboat_ws/src
git clone https://github.com/chenyuhang085712/2019-sailboat-ros.git
cd ~/sailboat_ws/src/2019-sailboat-ros/Sailboat-Ros/Doc/install
./install_ros.sh
```

#### 2. catkin build
```$xslt
cd ~/sailboat_ws
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin build
```
#### 3.tutorial

[class tutorial](https://github.com/hywel1994/Sailboat-Ros/blob/kinetic/Doc/tutorial_class.md)


