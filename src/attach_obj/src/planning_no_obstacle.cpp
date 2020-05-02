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

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
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

    // Get basic frame info
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Get current joint value
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;

    joint_values = move_group.getCurrentJointValues();

    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Planning to a pose goal

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.x = 0.5;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Found plan: %d", success );

    // Visualize the found plan

    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Next");
    move_group.move();

    ros::shutdown();
    return 0;
}


    // std::vector<double> joints;

    // joints.assign(joint_names.size(), 0.0);

    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints[i]);
    // }

