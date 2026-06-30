
#include "morpheus_spawner/include/spawner.hpp"

int main(int argc, char** argv)
{
  // Start ROS node
  rclcpp::init(argc, argv);
  auto spawner_node = std::make_shared<SpawnerNode>();
  spawner_node->initialize();
  //rclcpp::spin(spawner_node);
  //rclcpp::shutdown();
  //return 0;
  
  // Parse arguments
  std::vector<std::string> arguments(argv, argv + argc);
  std::string mesh_path;
  Eigen::Vector3d position;
  // std::fill(position.begin(), position.end(), 0);
  Eigen::Vector3d euler;
  // std::fill(euler.begin(), euler.end(), 0);
  std::string mode = "none";
  for (std::size_t i = 0; i < arguments.size(); i++) {
    std::string s = arguments[i];
    //RCLCPP_INFO_STREAM(this->get_logger(), s);
    if (s == "-mesh_path") {
      mesh_path = arguments[i+1];
    }
    if (s == "-x") {
      position[0] = std::stod(arguments[i+1]);
    }
    if (s == "-y") {
      position[1] = std::stod(arguments[i+1]);
    }
    if (s == "-z") {
      position[2] = std::stod(arguments[i+1]);
    }
    if (s == "-R") {
      euler[0] = std::stod(arguments[i+1]);
    }
    if (s == "-P") {
      euler[1] = std::stod(arguments[i+1]);
    }
    if (s == "-Y") {
      euler[2] = std::stod(arguments[i+1]);
    }
    if (s == "-mode") {
      mode = arguments[i+1];
    }
  }

  Eigen::Quaterniond quaternion = eulerToQuaternion(euler);

  Eigen::Vector3d scale = {0.0254, 0.0254, 0.0254};

  if (mode == "spawn") {
    moveit_msgs::msg::CollisionObject collision_object = spawner_node->create(mesh_path, scale=scale, position=position, quaternion=quaternion);
    spawner_node->spawn(spawner_node->attached_collision_object_vector_.size() - 1);
  }
  if (mode == "despawn") {
    spawner_node->despawn(spawner_node->attached_collision_object_vector_.size() - 1);
  }
  if (mode == "attach") {
    spawner_node->attach(spawner_node->attached_collision_object_vector_.size() - 1);
  }
  if (mode == "detach") {
    spawner_node->detach(spawner_node->attached_collision_object_vector_.size() - 1);
  }

  moveit_msgs::msg::CollisionObject sphere_object;
  sphere_object.header.frame_id = spawner_node->move_group_interface_->getPlanningFrame();
  sphere_object.id = "test_sphere";
  shape_msgs::msg::SolidPrimitive sphere_primitive;
  sphere_primitive.type = sphere_primitive.SPHERE;
  sphere_primitive.dimensions.resize(1);
  sphere_primitive.dimensions[0] = 0.2;
  sphere_object.primitives.resize(1);
  sphere_object.primitives[0] = sphere_primitive;
  sphere_object.pose.position.x = 0.0;
  sphere_object.pose.position.y = 0.85;
  sphere_object.pose.position.z = 0.95;
  sphere_object.pose.orientation.w = 1.0;
  sphere_object.operation = sphere_object.ADD;
  //spawner_node->save(sphere_object);
  //spawner_node->spawn(sphere_object);

  moveit_msgs::msg::CollisionObject mesh_object;
  mesh_object.header.frame_id = spawner_node->move_group_interface_->getPlanningFrame();
  mesh_object.id = "test_mesh";
  std::string test_mesh_path = "file:///root/ros2_ws/src/morpheus_description/meshes/collision/teapot.obj";
  const Eigen::Vector3d scale_eigen(0.05, 0.05, 0.05); // mm/inch
  shapes::Mesh* m = shapes::createMeshFromResource(test_mesh_path, scale_eigen);
  shape_msgs::msg::Mesh mesh_msg;
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  mesh_object.meshes.resize(2);
  mesh_object.meshes[0] = mesh_msg;
  mesh_object.meshes[1] = mesh_msg;

  //mesh_object.primitives.resize(1);
  //mesh_object.primitives[0] = sphere_primitive;

  mesh_object.pose.position.x = 0.3;
  mesh_object.pose.position.y = 0.35;
  mesh_object.pose.position.z = 0.95;
  mesh_object.pose.orientation.w = 1.0;
  
  mesh_object.mesh_poses.resize(2);
  mesh_object.mesh_poses[0].position.z = 0.2;
  mesh_object.mesh_poses[0].orientation.w = 1;
  mesh_object.mesh_poses[1].position.z = -0.2;
  mesh_object.mesh_poses[1].orientation.w = 1;

  //mesh_object.primitive_poses.resize(1);
  //mesh_object.primitive_poses[0].position.z = -0.2;
  //mesh_object.primitive_poses[0].orientation.w = 1;

  mesh_object.operation = mesh_object.ADD;
  RCLCPP_INFO_STREAM(spawner_node->get_logger(), "Test spawning object");
  spawner_node->save(mesh_object);
  spawner_node->spawn(0);
  //spawner_node->attach(0);

  //RCLCPP_INFO_STREAM(spawner_node->get_logger(), "Spawner Node spinning");

  spawner_node->spin();

  return 0;
}