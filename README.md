# catkin_ars408
# Ros development with canylst-II device , without peak-can
/// Only the src package is provided here, and the following command is used to create a ros workspace


mdkdir ~/home/(username)/catkin_ws/src
cd ..
catkin_make

// copy src to your ros worksapce
cp catkin catkin_ars408/src ~/home/(username)/catkin_ws

//compile
catkin_make

//run node
roscore
rosrun ars408 main

#If you want to visualize clusters, you need write subscriber node 
