## Learning ROS  
### Task 0  
Installing ROS  
http://wiki.ros.org/noetic/Installation/Ubuntu  
To Set Up environment everytime, run command  
~~~
source /opt/ros/noetic/setup.bash
~~~   
I can also run command   
~~~
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc  
source ~/.bashrc 
~~~  
But this is not recommended as it can cause problems with different versions.  
### Task 1  
**1. Create catkin workspace**  
**Building Catkin Workspace.**  
~~~  
$ mkdir -p ~/catkin_ws/src  
$ cd ~/catkin_ws/  
$ catkin_make  
~~~  
**Source new setup .sh file**  
~~~  
source devel/setup.bash  
~~~  
**make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.**  
~~~  
echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share  
~~~  
**2. Create a ROS package**  
create a package 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy:  
~~~
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
~~~  
**Building a catkin workspace and sourcing setup file**
~~~
cd ~/catkin_ws  
catkin_make  
~/catkin_ws/devel/setup.bash  
~~~  
**3. Launching Turlesim**
~ roscore ~  
open a new terminal window
~~~  
sudo apt-get install ros-$(rosversion -d)-turtlesim  
rosrun turtlesim turtlesim_node  
~~~  
open yet another terminal window to control the turtle  
~~~  
rosrun turtlesim turtle_teleop_key  
~~~  
to control the bot.  
**4. Talker/listener**
first, setup the bash file  
~~~  
source ./devel/setup.bash
~~~  
now, create python file for talker and listener  
dwonload default files from link at http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscribe  
make sure to download in /src/scripts folder.  
run  
~~~  
roscore  
cd ~/catkin_ws  
catkin_make  
rosrun beginner_tutorials talker.py  
rosrun beginner_tutorials listener.py  
~~~  
add relevant comments to the python script and make sure to use 3 different terminal windows for above commands.  


