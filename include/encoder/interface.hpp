/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ENCODER_INTERFACE_H
#define OPENVMP_ENCODER_INTERFACE_H

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "encoder/srv/position_get.hpp"
#include "encoder/srv/velocity_get.hpp"

namespace encoder {

class Node;

class Interface {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface();

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity;

  rclcpp::Service<encoder::srv::PositionGet>::SharedPtr position_get;
  rclcpp::Service<encoder::srv::VelocityGet>::SharedPtr velocity_get;

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Parameter interface_prefix_;
  rclcpp::Parameter param_readings_;
  rclcpp::Parameter param_overflow_;

  std::mutex readings_mutex_;
  double position_last_;
  double velocity_last_;
  double velocity_last_position_;
  std::chrono::high_resolution_clock::time_point velocity_last_time_;

  std::string get_prefix_();

  virtual void get_current_position_() = 0;
  virtual void get_current_velocity_();

  void position_get_handler_(
      const std::shared_ptr<encoder::srv::PositionGet::Request> request,
      std::shared_ptr<encoder::srv::PositionGet::Response> response);
  void velocity_get_handler_(
      const std::shared_ptr<encoder::srv::VelocityGet::Request> request,
      std::shared_ptr<encoder::srv::VelocityGet::Response> response);
  
 private:
  std::shared_ptr<std::thread> thread_;
  volatile bool do_stop_;
  std::chrono::duration<double, std::ratio<1L>> period_;

  void run_();
  void stop_();
};

}  // namespace encoder

#endif  // OPENVMP_ENCODER_INTERFACE_H
