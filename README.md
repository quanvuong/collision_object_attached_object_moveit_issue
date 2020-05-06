# Video illustration

An illustration of the problem is at illustration_of_problem.webm

To view the video, you have to download it.

I also uploaded the video below. You can see the issue from the 1 minute mark onwards.

https://vimeo.com/415361046


# Step to reproduce the problem with collision object and attached object passing through each other

roslaunch panda_moveit_config demo.launch

Add the motion planning plugin in Rviz and configure the plugin by changing the values of a few fields. This is described in step 1 of the quickstart in rviz tutorial http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

Run catkin_make in this directory 

rosrun attach_obj_melodic planning_with_obstacle_movegroup_api_panda

Press Next in Rviz until both the torus and the cylinder is added to the world, with the torus being attached to the robot

If the torus does not become attached, manually attach it to the link 'panda_hand' by clicking on the checkbox correponding to the torus under the Scene Objects tab in Rviz

Perform planning using the GUI in Rviz

