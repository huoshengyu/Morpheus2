#pragma once

#include <moveit/collision_detection/collision_detector_allocator.hpp>
#include <moveit/collision_distance_field/collision_env_hybrid.hpp>

#include <moveit_collision_distance_field_export.h>

namespace collision_detection
{

/** \brief Custom collision detector allocator for hybrid collision detection with custom parameters */
class MOVEIT_COLLISION_DISTANCE_FIELD_EXPORT MorpheusCollisionDetectorAllocatorHybrid
  : public collision_detection::CollisionDetectorAllocatorTemplate<  
      collision_detection::CollisionEnvHybrid,   
      MorpheusCollisionDetectorAllocatorHybrid>  
{  
public:  
  static const std::string NAME;
  double size_x_;
  double size_y_;
  double size_z_;
  Eigen::Vector3d origin_;
  bool use_signed_distance_field_;
  double resolution_;
  double collision_tolerance_;
  double max_propagation_distance_;
  double padding_;
  double scale_;

  MorpheusCollisionDetectorAllocatorHybrid(
    double size_x = 3.0,
    double size_y = 3.0,
    double size_z = 4.0,
    const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
    bool use_signed_distance_field = false,
    double resolution = 0.02,
    double collision_tolerance = 0.0,
    double max_propogation_distance = 1.0, 
    double padding = 0.0,
    double scale = 1.0)  
  {
    size_x_ = size_x;
    size_y_ = size_y;
    size_z_ = size_z;
    origin_ = origin;
    use_signed_distance_field_ = use_signed_distance_field;
    resolution_ = resolution;
    collision_tolerance_ = collision_tolerance;
    max_propagation_distance_ = max_propogation_distance;
    padding_ = padding;
    scale_ = scale;
  } 

  collision_detection::CollisionEnvPtr allocateEnv(  
    const moveit::core::RobotModelConstPtr& robot_model) const override  
  {  
    return std::make_shared<collision_detection::CollisionEnvHybrid>(  
      robot_model,   
      std::map<std::string, std::vector<collision_detection::CollisionSphere>>(),  
      size_x_, size_y_, size_z_,  
      origin_,  
      use_signed_distance_field_,  
      resolution_,  
      collision_tolerance_,  
      max_propagation_distance_,
      padding_,
      scale_);  
  } 
  
  CollisionEnvPtr allocateEnv(
    const WorldPtr& world, 
    const moveit::core::RobotModelConstPtr& robot_model) const override
  {
    return std::make_shared<CollisionEnvHybrid>(
      robot_model, 
      world,
      std::map<std::string, std::vector<collision_detection::CollisionSphere>>(),  
      size_x_, size_y_, size_z_,  
      origin_,  
      use_signed_distance_field_,  
      resolution_,  
      collision_tolerance_,  
      max_propagation_distance_,
      padding_,
      scale_);  
  }

  CollisionEnvPtr allocateEnv(
    const CollisionEnvConstPtr& orig, 
    const WorldPtr& world) const override
  {
    return std::make_shared<CollisionEnvHybrid>(
      dynamic_cast<const CollisionEnvHybrid&>(*orig), 
      world);
  }
};

const std::string collision_detection::MorpheusCollisionDetectorAllocatorHybrid::NAME("HYBRID");
}  // namespace collision_detection