// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shapes
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

// Eigen
#include <Eigen/Geometry>

// Local include
#include "attached_collision_object.h"

moveit_msgs::AttachedCollisionObject createAttachedCollisionObject(moveit_msgs::CollisionObject collision_object,
                                                            std::string link_name)
{
  // Create an attached collision object message
  moveit_msgs::AttachedCollisionObject attached_collision_object;

  // Fill in values
  attached_collision_object.link_name = link_name;
  attached_collision_object.object = collision_object;
  // attached_collision_object.touch_links = touch_links;
  // attached_collision_object.detach_posture = detach_posture;
  // attached_collision_object.weight = weight;

  // Note: message still needs to be published to correct topic after this
  return attached_collision_object;
}
