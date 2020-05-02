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

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
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

    // Visualization
    // ^^^^^^^^^^^^^
    //
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

    // Add the mug and the cylinder into planning scene

    // First create the mug
    shapes::Mesh *m1 = shapes::createMeshFromResource("package://attach_obj/meshes/mugblack_noground.stl");
    ROS_INFO_STREAM("mesh loaded from disk");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m1, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    moveit_msgs::CollisionObject mesh_collision_object;
    mesh_collision_object.header.frame_id = "world";
    mesh_collision_object.id = "mug";

    tf2::Quaternion mesh_q;
    mesh_q.setRPY(0.0, 0, -2.8);

    geometry_msgs::Pose mesh_pose;
    mesh_pose.position.x = 0.5;
    mesh_pose.position.y = 0.0;
    mesh_pose.position.z = 0.0;
    mesh_pose.orientation.x = mesh_q[0];
    mesh_pose.orientation.y = mesh_q[1];
    mesh_pose.orientation.z = mesh_q[2];
    mesh_pose.orientation.w = mesh_q[3];

    mesh_collision_object.meshes.push_back(mesh);
    mesh_collision_object.mesh_poses.push_back(mesh_pose);
    mesh_collision_object.operation = mesh_collision_object.ADD;

    // Second create the cylinder
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "cylinder1";

    // Define a box to add to the world.
    // The height and radius might be reversed
    // https://groups.google.com/forum/#!topic/moveit-users/LCn5BPK2h5o
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 1.0;  // height of the cylinder
    primitive.dimensions[1] = 0.01; // radius

    // Define a pose for the box (specified relative to frame_id)
    tf2::Quaternion my_quaternion;
    my_quaternion.setRPY(1.6, 0, 0);

    geometry_msgs::Pose box_pose;

    box_pose.orientation.x = my_quaternion[0];
    box_pose.orientation.y = my_quaternion[1];
    box_pose.orientation.z = my_quaternion[2];
    box_pose.orientation.w = my_quaternion[3];

    box_pose.position.x = 0.435;
    box_pose.position.y = -0.2;
    box_pose.position.z = 0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the mug and the cylinder into the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_vector;
    collision_vector.resize(2);
    collision_vector.push_back(mesh_collision_object);
    collision_vector.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_vector);

    // Show text in RViz of statusn
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Wait for MoveGroup to recieve and process the collision object message
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    ros::shutdown();
    return 0;
}
