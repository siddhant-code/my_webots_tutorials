/**
 * @file controller.cpp
 * @author Siddhant
 * @brief Implementaion of Controller node for e-puck robot 
 * @version 0.1
 * @date 2025-11-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "walker/controller.hpp"
#include <chrono>

using namespace std::chrono_literals;

//////////////////// Controller Implementation ////////////////////

Controller::Controller(float obstacle_threshold, float clear_threshold)
    : Node("walker_node"),
      obstacle_threshold_(obstacle_threshold),
      clear_threshold_(clear_threshold) {
  RCLCPP_INFO(this->get_logger(), "Initializing Walker Controller node");

  // Subscribe to LaserScan messages
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Controller::ScanCallback, this,
                             std::placeholders::_1));

  // Publisher for velocity commands
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // initial state is Forward
  current_state_ = std::make_shared<ForwardState>();
  RCLCPP_INFO(this->get_logger(), "Initial state set to ForwardState");

  // Timer for control loop at 20 Hz
  timer_ = this->create_wall_timer(
      50ms, std::bind(&Controller::ControlLoop, this));
}

// LaserScan callback: stores center beam distance for state logic.
void Controller::ScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  last_scan_ = msg->ranges[msg->ranges.size() / 2];
  RCLCPP_DEBUG(this->get_logger(), "LaserScan center reading: %.3f", last_scan_);
}

// Periodic control loop: delegates behavior to the active state.
void Controller::ControlLoop() {
  if (current_state_) {
    current_state_->Execute(this);
  } else {
    RCLCPP_WARN(this->get_logger(), "Current state is null in ControlLoop");
  }
}

// Changes the active behavior state (Forward or Rotate).
void Controller::SetState(std::shared_ptr<State> state) {
  if (state) {
    RCLCPP_INFO(this->get_logger(), "Transitioning to new state");
  } else {
    RCLCPP_WARN(this->get_logger(), "Attempted to set null state");
  }
  current_state_ = state;
}

// Computes the velocity command based on current state and scan input.
// Allows clean unit testing without ROS.
geometry_msgs::msg::Twist Controller::ComputeCmd(float scan) {
  geometry_msgs::msg::Twist cmd;
  // Safety check.
  if (current_state_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "ComputeCmd called with null current_state");
    return cmd;
  }

  // ForwardState logic.
  if (dynamic_cast<ForwardState*>(current_state_.get())) {
    // If obstacle too close , switch to RotateState.
    if (scan < obstacle_threshold_) {
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle detected at %.3f m, switching to RotateState",
                  scan);
      SetState(std::make_shared<RotateState>());
    } else {
      // Drive forward.
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.0;
      RCLCPP_DEBUG(this->get_logger(), "Moving forward, linear.x=%.2f", cmd.linear.x);
    }
  // RotateState logic.
  } else if (dynamic_cast<RotateState*>(current_state_.get())) {
    // Rotate clockwise or counter-clockwise.
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.2 * rotate_flag_;
    RCLCPP_DEBUG(this->get_logger(),
                 "Rotating in place, angular.z=%.2f", cmd.angular.z);
    // When path becomes clear , switch back to ForwardState.
    if (scan > clear_threshold_) {
      RCLCPP_INFO(this->get_logger(),
                  "Path clear at %.3f m, switching to ForwardState", scan);
      SetState(std::make_shared<ForwardState>());
      rotate_flag_ = -rotate_flag_; // Alternate direction each time.
    }
  }

  return cmd;
}

//////////////////// ForwardState Implementation ////////////////////

// ForwardState: robot moves forward unless obstacle detected.
void ForwardState::Execute(Controller* controller) {
  // Compute command based on scan.
  auto cmd = controller->ComputeCmd(controller->last_scan_);
  // Publish the velocity command.
  controller->cmd_pub_->publish(cmd);
  RCLCPP_DEBUG(controller->get_logger(), "Published ForwardState cmd: linear=%.2f angular=%.2f",
               cmd.linear.x, cmd.angular.z);
}

//////////////////// RotateState Implementation ////////////////////

// RotateState: robot rotates until scan distance exceeds clear threshold.
void RotateState::Execute(Controller* controller) {
  auto cmd = controller->ComputeCmd(controller->last_scan_);
  controller->cmd_pub_->publish(cmd);
  RCLCPP_DEBUG(controller->get_logger(), "Published RotateState cmd: linear=%.2f angular=%.2f",
               cmd.linear.x, cmd.angular.z);
}

//////////////////// Main ////////////////////

// Entry point: initializes ROS, starts node, then spins.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Walker Controller node");
  // Spin the controller node.
  rclcpp::spin(std::make_shared<Controller>());
  RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down Walker Controller node");
  rclcpp::shutdown();
  return 0;
}
