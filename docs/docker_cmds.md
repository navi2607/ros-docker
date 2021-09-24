### Build a image from the provided Dockerfile
**docker build -t ros:ubuntu_1804 .**

* build --> the docker command used tu build an image
* -t is an optional argument, used to name the image ros:ubuntu_1804
* . - use the Dockerfile from the current directory


### Temporary run a container
**docker run -it --rm ros:ubuntu_1804 bash**

* run --> docker command to run a container
* -it run in interactive mode
* --rm remove the container after exit
* name:tag the image name
* bash this is the command to run in the terminal inside the container


### Run roscore

```
root@bb95e0398369:/# roscore
... logging to /root/.ros/log/0e133346-13cd-11ec-800f-0242ac110002/roslaunch-bb95e0398369-36.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://bb95e0398369:39185/
ros_comm version 1.14.11


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.11

NODES

auto-starting new master
process[master]: started with pid [46]
ROS_MASTER_URI=http://bb95e0398369:11311/

setting /run_id to 0e133346-13cd-11ec-800f-0242ac110002
process[rosout-1]: started with pid [57]
started core service [/rosout]

```