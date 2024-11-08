#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "bot_msgs/action/temp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>



//  this file is used to give the goal to the moveit 
//  and acting as a interface between the moveit and alexa



using namespace std::placeholders;

namespace bot_remote
{
class TaskServer : public rclcpp::Node
{
  public:
    explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("task_server", options)
    {
      RCLCPP_INFO(get_logger(), "Starting the Server");
      action_server_ = rclcpp_action::create_server<bot_msgs::action::Temp>(
          this, "task_server", std::bind(&TaskServer::goalCallback,this, _1, _2),
          std::bind(&TaskServer::cancelCallback, this, _1),
          std::bind(&TaskServer::acceptedCallback, this, _1));
    }

  private:
    rclcpp_action::Server<bot_msgs::action::Temp>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const bot_msgs::action::Temp::Goal> goal)
    {
      RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<bot_msgs::action::Temp>> goal_handle)
    {
      (void)goal_handle;
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");

      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
      auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

      arm_move_group.stop();
      gripper_move_group.stop();

      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<bot_msgs::action::Temp>> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
    }


  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bot_msgs::action::Temp>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<bot_msgs::action::Temp::Result>();

    // MoveIt 2 Interface
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

    // **Fetch the current joint values directly**
    std::vector<double> current_arm_joint_values = arm_move_group.getCurrentJointValues();
    std::vector<double> current_gripper_joint_values = gripper_move_group.getCurrentJointValues();

    RCLCPP_INFO(get_logger(), "Current arm joint values: [%f, %f, %f]",
                current_arm_joint_values[0], current_arm_joint_values[1], current_arm_joint_values[2]);
    RCLCPP_INFO(get_logger(), "Current gripper joint values: [%f, %f]",
                current_gripper_joint_values[0], current_gripper_joint_values[1]);

    std::vector<double> arm_joint_goal;
    std::vector<double> gripper_joint_goal;

    // Set goal joint positions based on the task number
    if (goal_handle->get_goal()->task_number == 0)
    {
        arm_joint_goal = {0.0, 0.0, 0.0};
        gripper_joint_goal = {0.0, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 1)
    {
        arm_joint_goal = {-1.14, -0.6, -0.07};
        gripper_joint_goal = {0.0, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 2)
    {
        arm_joint_goal = {-1.57, 0.0, 1.0};
        gripper_joint_goal = {0.0, 0.0};
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Invalid Task Number");
        result->success = false;
        goal_handle->succeed(result);
        return;
    }

    // **Set the current joint values as the start state to ensure correct start position**
    arm_move_group.setJointValueTarget(arm_joint_goal);
    gripper_move_group.setJointValueTarget(gripper_joint_goal);

    // Plan for both arm and gripper
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (arm_plan_success && gripper_plan_success)
    {
        RCLCPP_INFO(get_logger(), "Planner succeeded, moving the arm and the gripper.");
        arm_move_group.move();
        gripper_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "One or more planners failed!");
        result->success = false;
        goal_handle->succeed(result);
        return;
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }


  };
}  // namespace bot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(bot_remote::TaskServer)
