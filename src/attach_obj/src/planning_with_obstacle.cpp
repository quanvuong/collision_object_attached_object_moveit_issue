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

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Init visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    // Get basic frame info
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Add the mug to the scene and attach to robot
    shapes::Mesh *m1 = shapes::createMeshFromResource("package://attach_obj/meshes/torus_small.stl");
    ROS_INFO_STREAM("mesh loaded from disk");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m1, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    moveit_msgs::AttachedCollisionObject mesh_collision_object;
    mesh_collision_object.link_name = "wrist_3_link";
    mesh_collision_object.object.header.frame_id = "wrist_3_link";
    mesh_collision_object.object.id = "mug";

    tf2::Quaternion mesh_q;
    mesh_q.setRPY(0.0, 0, 0.0);

    geometry_msgs::Pose mesh_pose;
    mesh_pose.position.x = 0.0;
    mesh_pose.position.y = 0.15;
    mesh_pose.position.z = 0.0;
    mesh_pose.orientation.x = mesh_q[0];
    mesh_pose.orientation.y = mesh_q[1];
    mesh_pose.orientation.z = mesh_q[2];
    mesh_pose.orientation.w = mesh_q[3];

    mesh_collision_object.object.meshes.push_back(mesh);
    mesh_collision_object.object.mesh_poses.push_back(mesh_pose);
    mesh_collision_object.object.operation = mesh_collision_object.object.ADD;

    mesh_collision_object.touch_links = std::vector<std::string>{
        "wrist_3_link", "wrist_2_link", "ee_link"
    };

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state.attached_collision_objects.push_back(mesh_collision_object);
    planning_scene.is_diff = true;

    ros::ServiceClient planning_scene_diff_client =
        node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
    // and send the diffs to the planning scene via a service call:
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);

    move_group.attachObject(mesh_collision_object.object.id);

    ROS_INFO("Add cylinder");
    // Add the cylinder into the scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "wrist_3_link";
    collision_object.id = "cylinder1";

    // Define a cylinder to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 1.0;  // height of the cylinder
    primitive.dimensions[1] = 0.01; // radius

    // Define a pose for the cylinder (specified relative to frame_id)
    tf2::Quaternion my_quaternion;
    my_quaternion.setRPY(0.0, 0, 0);

    geometry_msgs::Pose cyl_pose;

    cyl_pose.orientation.x = my_quaternion[0];
    cyl_pose.orientation.y = my_quaternion[1];
    cyl_pose.orientation.z = my_quaternion[2];
    cyl_pose.orientation.w = my_quaternion[3];

    cyl_pose.position.x = 0.0;
    cyl_pose.position.y = 0.15;
    cyl_pose.position.z = 0.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cyl_pose);
    collision_object.operation = collision_object.ADD;

    // Add the mug and the cylinder into the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);

    // // Planning to a pose goal
    // geometry_msgs::Pose target_pose;
    // target_pose.orientation.w = 1.0;
    // target_pose.position.x = 0.5;
    // target_pose.position.y = -0.0;
    // target_pose.position.z = 0.0;

    // move_group.setPoseTarget(target_pose);

    // moveit::planning_interface::MoveGroupInterface::Plan plan;

    // bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO("Found plan: %s", success ? "true" : "false");

    // move_group.move();
    visual_tools.prompt("Next to end");

    ros::shutdown();
    return 0;
}


    // std::vector<double> joints;

    // joints.assign(joint_names.size(), 0.0);

    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints[i]);
    // }

// Planning to a joint space goal
// moveit::core::RobotModelPtr current_state = move_group.getCurrentState();
// joint_values.assign(joint_names.size(), 0.001);

// move_group.setJointValueTarget(joint_values);

// moveit::planning_interface::MoveGroupInterface::Plan plan;

// bool success = (move_group.plan(plan)
//     == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// ROS_INFO("Found plan: %s", success ? "true" : "false" );

// // Get current joint value
// const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
// std::vector<double> joint_values;

// joint_values = move_group.getCurrentJointValues();

// for (std::size_t i = 0; i < joint_names.size(); ++i)
// {
//     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
// }
