#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_with_obstacle_move_group_api_panda");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    // Getting Basic Information
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Add the torus into the world
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    shapes::Mesh *m1 = shapes::createMeshFromResource("package://attach_obj/meshes/torus_small.stl");
    ROS_INFO_STREAM("mesh loaded from disk");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m1, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    moveit_msgs::CollisionObject mesh_collision_object;
    mesh_collision_object.header.frame_id = "panda_leftfinger";
    mesh_collision_object.id = "mug";

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

    mesh_collision_object.meshes.push_back(mesh);
    mesh_collision_object.mesh_poses.push_back(mesh_pose);
    mesh_collision_object.operation = mesh_collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> mesh_collision_objects;
    mesh_collision_objects.push_back(mesh_collision_object);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(mesh_collision_objects);

    // Wait for MoveGroup to recieve and process the collision object message
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group.attachObject(mesh_collision_object.id);

    /* Wait for MoveGroup to recieve and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                        "robot");

    // Add the cylinder into the world
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "panda_leftfinger";
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

    planning_scene_interface.applyCollisionObject(collision_object);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("tutorial", "Add a cylinder into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.prompt("Ready to perform motion planning using GUI after the cylinder is added to the world");

    ros::shutdown();
    return 0;
}
