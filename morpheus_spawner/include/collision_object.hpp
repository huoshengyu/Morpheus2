moveit_msgs::msg::CollisionObject createCollisionObject(std::string, 
                                                  std::vector<double>,
                                                  std::vector<double>,
                                                  std::vector<double>);

moveit_msgs::msg::CollisionObject createCollisionObjectFromMap(std::string, 
                                                  std::map<std::string, double>,
                                                  std::map<std::string, double>,
                                                  std::map<std::string, double>);

moveit_msgs::msg::CollisionObject addCollisionObject(moveit::planning_interface::PlanningSceneInterface,
                                                  std::string, 
                                                  std::vector<double>,
                                                  std::vector<double>,
                                                  std::vector<double>);

moveit_msgs::msg::CollisionObject addCollisionObjectFromMap(moveit::planning_interface::PlanningSceneInterface,
                                                  std::string, 
                                                  std::map<std::string, double>,
                                                  std::map<std::string, double>,
                                                  std::map<std::string, double>);