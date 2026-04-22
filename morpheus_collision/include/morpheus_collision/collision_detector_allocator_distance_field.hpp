#pragma once

#include <moveit/collision_detection/collision_detector_allocator.hpp>
#include <moveit/collision_distance_field/collision_env_distance_field.hpp>

namespace collision_detection
{
static const double DEFAULT_SIZE_X = 3.0;
static const double DEFAULT_SIZE_Y = 3.0;
static const double DEFAULT_SIZE_Z = 4.0;
static const Eigen::Vector3d DEFAULT_ORIGIN(0, 0, 0); 
static const bool DEFAULT_USE_SIGNED_DISTANCE_FIELD = false;
static const double DEFAULT_RESOLUTION = .02;
static const double DEFAULT_COLLISION_TOLERANCE = 0.0;
static const double DEFAULT_MAX_PROPOGATION_DISTANCE = 1.0;
static const double DEFAULT_PADDING = 0.0;
static const double DEFAULT_SCALE = 1.0;

/** \brief Custom collision detector allocator for distance field-based collision detection with custom parameters */
class MorpheusCollisionDetectorAllocatorDistanceField   
  : public collision_detection::CollisionDetectorAllocatorTemplate<  
      collision_detection::CollisionEnvDistanceField,   
      MorpheusCollisionDetectorAllocatorDistanceField>  
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

  MorpheusCollisionDetectorAllocatorDistanceField(
    double size_x = DEFAULT_SIZE_X,
    double size_y = DEFAULT_SIZE_Y,
    double size_z = DEFAULT_SIZE_Z,
    const Eigen::Vector3d& origin = DEFAULT_ORIGIN,
    bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
    double resolution = DEFAULT_RESOLUTION,
    double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
    double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE, 
    double padding = DEFAULT_PADDING,
    double scale = DEFAULT_SCALE)  
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
    return std::make_shared<collision_detection::CollisionEnvDistanceField>(  
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
    return std::make_shared<CollisionEnvDistanceField>(
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
    return std::make_shared<CollisionEnvDistanceField>(
      dynamic_cast<const CollisionEnvDistanceField&>(*orig), 
      world);
  }
};

const std::string collision_detection::MorpheusCollisionDetectorAllocatorDistanceField::NAME("DISTANCE_FIELD");
}  // namespace collision_detection