#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
    
  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -2.50; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = -1.50; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position using joint value target");

  joint_group_positions_arm[0] = -0.4532;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.48353; // Shoulder Lift
  joint_group_positions_arm[2] = 1.67552;  // Elbow
  joint_group_positions_arm[3] = -1.74533; // Wrist 1
  joint_group_positions_arm[4] = -1.5708;  // Wrist 2
  joint_group_positions_arm[5] = -2.02458; // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);


  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("gripper_open");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  joint_group_positions_arm[0] = -0.4536; // Shoulder Pan
  joint_group_positions_arm[1] = -1.3326; // Shoulder Lift
  joint_group_positions_arm[2] = 1.96773; // Elbow
  joint_group_positions_arm[3] = -2.2062; // Wrist 1
  joint_group_positions_arm[4] = -1.5717; // Wrist 2
  joint_group_positions_arm[5] = -2.0234; // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);
  std::this_thread::sleep_for(std::chrono::seconds(2));


  float gripper_value = 0.60;
  while (gripper_value <= 0.645) {
    joint_group_positions_gripper[2] = gripper_value;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
    RCLCPP_INFO(LOGGER, "Closing gripper: %.3f.", gripper_value);
    success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                       moveit::core::MoveItErrorCode::SUCCESS);
    if (success_gripper) {
      RCLCPP_INFO(LOGGER, "Plan to actuate gipper: SUCCESS.");
      RCLCPP_INFO(LOGGER, "Executing command.");
      move_group_arm.execute(my_plan_gripper);
      gripper_value += 0.001;
    } else {
      RCLCPP_INFO(LOGGER, "Plan to actuate gipper: FAIL.");
      RCLCPP_INFO(LOGGER, "Aborting command.");
      rclcpp::shutdown();
      return 1;
    }
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");


  joint_group_positions_arm[0] = -0.4532;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.48353; // Shoulder Lift
  joint_group_positions_arm[2] = 1.67552;  // Elbow
  joint_group_positions_arm[3] = -1.74533; // Wrist 1
  joint_group_positions_arm[4] = -1.5708;  // Wrist 2
  joint_group_positions_arm[5] = -2.02458; // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Place

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] = 2.82743;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.48353; // Shoulder Lift
  joint_group_positions_arm[2] = 1.67552;  // Elbow
  joint_group_positions_arm[3] = -1.74533; // Wrist 1
  joint_group_positions_arm[4] = -1.5708;  // Wrist 2
  joint_group_positions_arm[5] = -2.02458; // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("gripper_open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  rclcpp::shutdown();
  return 0;
}