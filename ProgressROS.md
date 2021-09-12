##Learning ROS  
###Task 0  
Installing ROS  
http://wiki.ros.org/noetic/Installation/Ubuntu  
To Set Up environment everytime, run command  
~source /opt/ros/noetic/setup.bash~  
I can also run command   
~~~
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc  
source ~/.bashrc 
~~~  
But this is not recommendas it can cause problems with different versions.  
###Task 1  
**1. Create catkin workspace**  
**Building Catkin Workspace.**  
~~~  
$ mkdir -p ~/catkin_ws/src  
$ cd ~/catkin_ws/  
$ catkin_make  
~~~  
**Source new setup .sh file**  
~source devel/setup.bash~  
**make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.**  
~~~  
echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share  
~~~  

