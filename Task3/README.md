# Working with TurtleSim
First of All, Create a catkin package  
~~~  
cd ~/catkin_ws/src  
catkin_create_pkg turtlesim_cleaner  
catkin_make  
#run roscore
roscore
#open new terminal window
rosrun turtlesim turtlesim_node
#open new terminal window
rosrun turtlesim_cleaner st_line.py  
~~~  
