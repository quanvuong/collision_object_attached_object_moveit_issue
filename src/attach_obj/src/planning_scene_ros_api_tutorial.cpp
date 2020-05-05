#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_scene_ros_api_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle;

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    /* Attach an object to the robot */
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "panda_leftfinger";
    attached_object.object.header.frame_id = "panda_leftfinger";
    attached_object.object.id = "box";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 0.1;
    // primitive.dimensions[1] = 0.1;
    // primitive.dimensions[2] = 0.1;

    // attached_object.object.primitives.push_back(primitive);
    // attached_object.object.primitive_poses.push_back(pose);
    // attached_object.object.operation = attached_object.object.ADD;

    shapes::Mesh *m1 = shapes::createMeshFromResource("package://attach_obj/meshes/torus.stl");
    ROS_INFO_STREAM("mesh loaded from disk");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m1, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    geometry_msgs::Pose mesh_pose;
    mesh_pose.orientation.w = 1.0;

    attached_object.object.meshes.push_back(mesh);
    attached_object.object.mesh_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    attached_object.touch_links = std::vector<std::string>{"panda_hand", "panda_leftfinger", "panda_rightfinger"};
    
    ROS_INFO("Attaching the object to the hand.");
    moveit_msgs::PlanningScene planning_scene;

    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_diff_publisher.publish(planning_scene);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Detach an object from the robot and remove it from the world

    /* First, define the DETACH object message*/
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "box";
    detach_object.link_name = "panda_link8";
    detach_object.object.operation = attached_object.object.REMOVE;

    // Define remove msg
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = "box";
    remove_object.header.frame_id = "panda_link0";
    remove_object.operation = remove_object.REMOVE;

    /* Carry out the DETACH + ADD operation */
    ROS_INFO("Detaching the object from the robot and removing it from the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.robot_state.is_diff = true;
    
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.is_diff = true;
    
    planning_scene_diff_publisher.publish(planning_scene);

    visual_tools.prompt("Next");

    ros::shutdown();
    return 0;
}
