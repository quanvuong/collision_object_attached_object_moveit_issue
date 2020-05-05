#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "panda_arm";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

    // Adding/Removing Objects and Attaching/Detaching Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Define a collision object ROS message.
    // Add the mug to the scene and attach to robot
    shapes::Mesh *m1 = shapes::createMeshFromResource("package://attach_obj/meshes/mugblack_noground.stl");
    ROS_INFO_STREAM("mesh loaded from disk");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m1, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "mug";

    geometry_msgs::Pose mesh_pose;
    mesh_pose.position.x = 0.4;
    mesh_pose.position.y = -0.2;
    mesh_pose.position.z = 1.0;
    mesh_pose.orientation.w = 1.0;

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    success = planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO_NAMED("tutorial", "appply collision object: %s", success ? "true" : "false");

    // Show text in RViz of status
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Wait for MoveGroup to recieve and process the collision object message
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // Now when we plan a trajectory it will avoid the obstacle
    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::Pose another_pose;
    another_pose.orientation.w = 1.0;
    another_pose.position.x = 0.4;
    another_pose.position.y = -0.4;
    another_pose.position.z = 0.9;
    move_group.setPoseTarget(another_pose);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Now, let's attach the collision object to the robot.
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group.attachObject(collision_object.id);

    // Show text in RViz of status
    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to recieve and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                        "robot");

    // Now, let's detach the collision object from the robot.
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    move_group.detachObject(collision_object.id);

    // Show text in RViz of status
    visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to recieve and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
                        "robot");

    // Now, let's remove the collision object from the world.
    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    // Show text in RViz of status
    visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to recieve and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

    // END_TUTORIAL

    ros::shutdown();
    return 0;
}
 