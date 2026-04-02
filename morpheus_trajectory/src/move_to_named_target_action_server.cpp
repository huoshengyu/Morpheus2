// ROS Imports
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
// ROS Messages
#include <moveit_msgs/msg/move_it_error_codes.hpp>
// Moveit
#include <moveit/move_group_interface/move_group_interface.hpp>
// ROS2 Control
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/msg/controller_state.hpp>
// Universal Robots Driver
// #include <ur_msgs/action/follow_joint_trajectory_until.hpp>
// Local Imports
#include "morpheus_msgs/action/move_to_named_target.hpp"
#include "morpheus_trajectory/visibility_control.hpp"

namespace morpheus_trajectory
{
class MoveToNamedTargetActionServer : public rclcpp::Node
{
public:
    // Set aliases for readability
    using Action = morpheus_msgs::action::MoveToNamedTarget;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    using ListControllers = controller_manager_msgs::srv::ListControllers;
    using SwitchController = controller_manager_msgs::srv::SwitchController;
    using ControllerState = controller_manager_msgs::msg::ControllerState;

    // Move group names
    std::string arm_group_;
    std::string gripper_group_;

    // Controller names
    std::vector<std::string> conflicting_controllers_;
    std::string trajectory_controller_;

    // Controller service clients
    rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
    rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;

    // Interfaces
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;

    // Action server
    rclcpp_action::Server<Action>::SharedPtr action_server_;
    
    MORPHEUS_TRAJECTORY_PUBLIC
    explicit MoveToNamedTargetActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
        : Node("move_to_named_target_action_server", options)
    {
        // Use placeholders namespace to shorten calls to _1, _2 in callback bindings
        using namespace std::placeholders;

        // Declare parameters
        this->declare_parameter("arm_group", "arm");
        this->declare_parameter("gripper_group", "gripper");
        this->declare_parameter("trajectory_controller", "scaled_joint_trajectory_controller");

        // Get arm and gripper groups from ros server, if possible
        arm_group_ = this->get_parameter("arm_group").as_string();
        gripper_group_ = this->get_parameter("gripper_group").as_string();
        trajectory_controller_ = this->get_parameter("trajectory_controller").as_string();
        conflicting_controllers_ = {};

        // Initialize controller service clients
        list_controllers_client_ = 
            this->create_client<ListControllers>("/controller_manager/list_controllers");
        switch_controller_client_ = 
            this->create_client<SwitchController>("/controller_manager/switch_controller");
        
        // Initialize the move group interface so that planning can be conducted
        move_group_interface_ = nullptr;
        plan_ = moveit::planning_interface::MoveGroupInterface::Plan();

        // Initialize the action server so targets can be received
        action_server_ = rclcpp_action::create_server<Action>(
            this,
            "move_to_named_target",
            std::bind(&MoveToNamedTargetActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveToNamedTargetActionServer::handle_cancel, this, _1),
            std::bind(&MoveToNamedTargetActionServer::handle_accepted, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Waiting for controller manager services");
        list_controllers_client_->wait_for_service();
        switch_controller_client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Controller manager services found");
    }

private:
    // Declare functions needed to switch controllers
    // Get the list of controllers from the controller manager
    std::vector<ControllerState> get_controllers_list()
    {
        auto request = std::make_shared<ListControllers::Request>();
        auto result_future = list_controllers_client_->async_send_request(request);
        while (rclcpp::ok())
        {
            if (result_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) 
            {
                break;
            }
        }
        auto result = result_future.get();
        return result->controller;
    }

    // Switch controllers
    bool switch_controller(
        std::vector<std::string> activate_controllers = {}, 
        std::vector<std::string> deactivate_controllers = {})
    {
        auto request = std::make_shared<SwitchController::Request>();
        request->activate_controllers = activate_controllers;
        request->deactivate_controllers = deactivate_controllers;
        request->strictness = request->STRICT;
        request->activate_asap = true;

        std::stringstream activate_controllers_ss;
        for (std::string controller_name : activate_controllers)
        {
            activate_controllers_ss << controller_name << ", ";
        }
        RCLCPP_INFO(this->get_logger(), "Activating controllers: {%s}", activate_controllers_ss.str().c_str());
        std::stringstream deactivate_controllers_ss;
        for (std::string controller_name : deactivate_controllers)
        {
            deactivate_controllers_ss << " " << controller_name << ",";
        }
        RCLCPP_INFO(this->get_logger(), "Deactivating controllers: {%s}", deactivate_controllers_ss.str().c_str());

        auto result_future = switch_controller_client_->async_send_request(request);
        while (rclcpp::ok())
        {
            if (result_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) 
            {
                break;
            }
        }
        auto result = result_future.get();
        if (result->ok)
        {
            RCLCPP_INFO(this->get_logger(), "Switching controllers succeeded");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Switching controllers failed");
        }
        return result->ok;
    }

    // Get a ControllerState message by name from a list of ControllerState messages
    ControllerState get_controller(
        std::vector<ControllerState> controllers_list,
        std::string controller_name)
    {
        for (ControllerState controller : controllers_list)
        {
            if (controller.name == controller_name)
            {
                return controller;
            }
        }
        return ControllerState();
    }

    // Get the names of the controllers in a controller list
    std::vector<std::string> get_controller_names(std::vector<ControllerState> controllers_list)
    {
        std::vector<std::string> names = {};
        for (ControllerState controller : controllers_list)
        {
            names.push_back(controller.name);
        }
        return names;
    }

    // Get controllers which claim interfaces required by the named controller
    std::vector<ControllerState> get_conflicting_controllers(
        std::vector<ControllerState> controllers_list,
        std::string controller_name)
    {
        ControllerState target_controller = get_controller(controllers_list, controller_name);
        std::vector<std::string> required_command_interfaces = target_controller.required_command_interfaces;
        std::vector<ControllerState> conflicting_controllers = {};
        for (ControllerState controller : controllers_list)
        {
            std::vector<std::string> claimed_interfaces = controller.claimed_interfaces;
            for (std::string command_interface : required_command_interfaces)
            {
                if (std::find(
                    claimed_interfaces.begin(), 
                    claimed_interfaces.end(), 
                    command_interface) 
                    != claimed_interfaces.end())
                {
                    conflicting_controllers.push_back(controller);
                    break;
                }
            }
        }
        return conflicting_controllers;
    }

    // Swap to trajectory controller, deactivating other controllers as needed
    bool to_trajectory_controller()
    {
        auto controllers_list = get_controllers_list();
        auto conflicting_controllers_list = get_conflicting_controllers(controllers_list, trajectory_controller_);
        conflicting_controllers_ = get_controller_names(conflicting_controllers_list);
        std::vector<std::string> trajectory_controllers = {trajectory_controller_}; // Need vector type as input
        return switch_controller(trajectory_controllers, conflicting_controllers_);
    }

    // Swap from trajectory controller, reactivating other controllers as needed
    bool from_trajectory_controller()
    {
        std::vector<std::string> trajectory_controllers = {trajectory_controller_}; // Need vector type as input
        return switch_controller(conflicting_controllers_, trajectory_controllers);
    }

    // Declare functions needed for the action server
    // Process action goals
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Action::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target_name %s", goal->target_name.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    // Process action cancels
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    // Process action acceptances
    void handle_accepted(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&MoveToNamedTargetActionServer::execute, this, _1), goal_handle}.detach();
    };

    // Execute action
    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Move to named target initiated");
        if (move_group_interface_ == nullptr)
        {
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), arm_group_);
            RCLCPP_INFO(this->get_logger(), "Move group interface initialized");
        }
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Action::Feedback>();
        auto & error_code = feedback->error_code;
        auto result = std::make_shared<Action::Result>();

        // Swap to trajectory controller
        to_trajectory_controller();

        // Set plan to go from current state to named target state
        std::vector<std::string> trajectory_controllers = {trajectory_controller_}; // Need vector type as input
        move_group_interface_->setStartStateToCurrentState();
        move_group_interface_->setNamedTarget(goal->target_name);

        // Move and update feedback response
        error_code = move_group_interface_->plan(plan_);
        error_code = move_group_interface_->execute(plan_, trajectory_controllers);

        // Swap back from trajectory controller
        from_trajectory_controller();

        // Publish feedback
        goal_handle->publish_feedback(feedback);

        // Publish result
        if (rclcpp::ok()) {
            result->error_code = error_code;
            if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Move to named target succeeded");
            }
            else
            {
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "Move to named target failed");
            }
        }
    };
};  // class MoveToNamedTargetActionServer

}  // namespace morpheus_trajectory

RCLCPP_COMPONENTS_REGISTER_NODE(morpheus_trajectory::MoveToNamedTargetActionServer);

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<morpheus_trajectory::MoveToNamedTargetActionServer>();
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    action_server = nullptr;
    return 0;
}