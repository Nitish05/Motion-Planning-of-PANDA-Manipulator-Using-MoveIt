#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char* argv[]) {
  // Initializing ROS 
  rclcpp::init(argc, argv);
  // Creating ROS node
  auto node = std::make_shared<rclcpp::Node>("moveit_project");

  // Setting up ROS logger.
  auto logger = rclcpp::get_logger("moveit_project");

  // Creating Move Group Interfaces for arm and hand gripper
  moveit::planning_interface::MoveGroupInterface arm_panda_move_group(node, "arm_pandas_group");
  moveit::planning_interface::MoveGroupInterface hand_panda_move_group(node, "hand_pandas_group");
  
  // Configuration for initial pose
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 0.0995571;  
  initial_pose.position.y = 0.00470973;
  initial_pose.position.z = 1.13997;
  initial_pose.orientation.w = 0.680195; 
  initial_pose.orientation.x = 0.269352;
  initial_pose.orientation.y = 0.56757;
  initial_pose.orientation.z = 0.377688;
  
  // Assigning the initial pose.
  arm_panda_move_group.setPoseTarget(initial_pose);
  
  // Executing the plan to move to initial pose.
  moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
  if (arm_panda_move_group.plan(initial_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      arm_panda_move_group.execute(initial_plan);
  } else {
      RCLCPP_ERROR(logger, "Initial Pose plan failed!");
      return 1;  
  }

  // Opening the hand gripper.
  hand_panda_move_group.setNamedTarget("opened_gripper");
  moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan;
  if (hand_panda_move_group.plan(open_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    hand_panda_move_group.execute(open_gripper_plan);
  } else {
    RCLCPP_ERROR(logger, "Open gripper plan failed!");
    return 1; 
  }

  // Configuration of pickup pose.
  geometry_msgs::msg::Pose pickup_pose;
  pickup_pose.position.x = 0.570485;  
  pickup_pose.position.y = 0.550141;
  pickup_pose.position.z = 0.304284;
  pickup_pose.orientation.w = 0.144854;
  pickup_pose.orientation.x = 0.0336483;
  pickup_pose.orientation.y = 0.964015;
  pickup_pose.orientation.z = 0.220362;
  
  // Assigning the pickup pose.
  arm_panda_move_group.setPoseTarget(pickup_pose);
  
  // Executing the plan to move to pickup pose. 
  moveit::planning_interface::MoveGroupInterface::Plan pickup_plan;
  if (arm_panda_move_group.plan(pickup_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm_panda_move_group.execute(pickup_plan);
	
    // Closing the hand gripper.
    hand_panda_move_group.setNamedTarget("closed_gripper");
    moveit::planning_interface::MoveGroupInterface::Plan close_gripper_plan;
    if (hand_panda_move_group.plan(close_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      hand_panda_move_group.execute(close_gripper_plan);
    } else {
      RCLCPP_ERROR(logger, "Close gripper plan failed!");
      return 1; 
    }
  } else {
    RCLCPP_ERROR(logger, "Pickup object plan failed!");
    return 1; 
  }
  
  // Configuration to place the object.
  geometry_msgs::msg::Pose place_pose;
  place_pose.position.x = 0.426611;  
  place_pose.position.y = -0.278568;
  place_pose.position.z = 0.268575;
  place_pose.orientation.w = 0.0125238;
  place_pose.orientation.x = 0.831042;
  place_pose.orientation.y = -0.556021;
  place_pose.orientation.z = -0.00725777;
  
  // Assigning the place pose to be planned.
  arm_panda_move_group.setPoseTarget(place_pose);

  // Exectuing the plan to move to place pose.
  moveit::planning_interface::MoveGroupInterface::Plan place_plan;
  if (arm_panda_move_group.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm_panda_move_group.execute(place_plan);
  } else {
    RCLCPP_ERROR(logger, "Place plan failed!");
    return 1; 
  }

  // Opening the hand gripper to release the object.
  hand_panda_move_group.setNamedTarget("opened_gripper");
  moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan_2;
  if (hand_panda_move_group.plan(open_gripper_plan_2) == moveit::core::MoveItErrorCode::SUCCESS) {
    hand_panda_move_group.execute(open_gripper_plan_2);
  } else {
    RCLCPP_ERROR(logger, "open gripper plan 2 failed!");
    return 1; 
  }
  
  // Configuration for returning to default position.
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = 0.0995571;  
  end_pose.position.y = 0.00470973;
  end_pose.position.z = 1.13997;
  end_pose.orientation.w = 0.680195; 
  end_pose.orientation.x = 0.269352;
  end_pose.orientation.y = 0.567657;
  end_pose.orientation.z = 0.377688;
  
  // Assigning the end pose to be planned.
  arm_panda_move_group.setPoseTarget(end_pose);

  // Executing Plan to move back to initial position.
  moveit::planning_interface::MoveGroupInterface::Plan end_plan;
  if (arm_panda_move_group.plan(end_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm_panda_move_group.execute(end_plan);
  } else {
    RCLCPP_ERROR(logger, "end state plan failed!");
    return 1; 
  }

  // Shutting down ROS
  rclcpp::shutdown();
  return 0;
}