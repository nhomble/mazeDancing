# Installation
```
$ cd [your groovy ROS workspace directory]
$ git clone https://github.com/nhomble/mazeDancing
$ git branch mazeSolver
$ git pull origin mazeSolver
# apt-get install ros-groovy 
# apt-get install ros-groovy-openni 
# apt-get install ros-groovy-ar-track-alvar 
# apt-get install ros-groovy-depthimage-to-laserscan 
$ rospack profile
```
# Running

On the scout robot open a terminal for each bash statement.
```
$ roscore
$ roslaunch turtlebot_bringup minimal.launch
$ roslaunch openni_launch openni.launch
$ roslaunch mazeDancing scout.launch
```

On the scoutLeader robot open a terminal for each bash statement.
```
$ roslaunch turtlebot_bringup minimal.launch
$ roslaunch openni_launch openni.launch
$ roslaunch mazeDancing scoutLeader.launch
```
