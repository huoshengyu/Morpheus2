#include "morpheus_spawner/spawner.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto spawner_node = std::make_shared<SpawnerNode>();
  spawner_node->initialize();
  rclcpp::spin(spawner_node);
  rclcpp::shutdown();
  return 0;
}
