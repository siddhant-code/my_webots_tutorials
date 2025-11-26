/**
 * @file controller.hpp
 * @author Siddhant
 * @brief Controller node for e-puck robot implementing Roomba-like walker
 *        behavior using State Design Pattern.
 * @version 0.1
 * @date 2025-11-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <functional>

/**
 * @brief Forward declaration of Controller class for State interface.
 */
class Controller;

/**
 * @brief Abstract State class for walker behavior.
 *
 * Designed so Execute() can be unit tested independently.
 */
class State {
 public:
  virtual ~State() = default;

  /**
   * @brief Execute the state's behavior.
   * @param controller Pointer to Controller node
   */
  virtual void Execute(Controller* controller) = 0;
};

/**
 * @brief Forward state: moves the robot forward until an obstacle is detected.
 */
class ForwardState : public State {
 public:
  void Execute(Controller* controller) override;
};

/**
 * @brief Rotate state: rotates the robot until the path ahead is clear.
 */
class RotateState : public State {
 public:
  void Execute(Controller* controller) override;
};

/**
 * @brief ROS2 Node implementing walker behavior with State pattern.
 *
 * Designed to be testable without ROS2:
 *   - Logic is independent of ROS2 calls
 *   - Velocity outputs can be captured for verification
 */
class Controller : public rclcpp::Node {
 public:
  /**
   * @brief Construct a Controller node.
   * @param obstacle_threshold Distance to trigger rotation (default 0.3)
   * @param clear_threshold Distance to switch back to forward (default 0.8)
   */
  Controller(float obstacle_threshold = 0.3f, float clear_threshold = 0.8f);

  /**
   * @brief Callback for LaserScan messages.
   * @param msg Latest LaserScan message.
   */
  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Control loop called periodically by timer.
   */
  void ControlLoop();

  /**
   * @brief Change the current state.
   * @param state New state.
   */
  void SetState(std::shared_ptr<State> state);

  /**
   * @brief Compute the velocity command given the current state and sensor input.
   *
   * Useful for unit testing: does not publish, just returns Twist.
   */
  geometry_msgs::msg::Twist ComputeCmd(float scan);

  /// Last center scan reading
  float last_scan_ = 1.0f;

  /// Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  /// Alternates rotation direction between clockwise and counter-clockwise
  int rotate_flag_ = 1;

  /// Obstacle detection threshold (meters)
  float obstacle_threshold_;

  /// Clear path threshold (meters)
  float clear_threshold_;

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// Current state of the walker
  std::shared_ptr<State> current_state_;
};

#endif  // CONTROLLER_HPP_
