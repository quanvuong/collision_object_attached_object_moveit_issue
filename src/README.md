Move group Interface doc: http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

# Step to reproduce the problem with collision object and attached object passing through each other

roslaunch panda_moveit_config demo.launch

roslaunch moveit_tutorials move_group_interface_tutorial.launch

Set the fields in Rviz correct, following the tutorial

http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html
(the part just above Step 2)

Run planning_with_obstacle_movegroup_api_panda

Press Next in Rviz until both the torus and the cylinder is added to the world, with the torus being attached to the robot

Perform planning using the GUI in Rviz