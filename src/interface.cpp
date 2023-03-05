/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "encoder/interface.hpp"

#include <functional>

namespace encoder {

Interface::Interface(rclcpp::Node *node)
    : node_{node} {
  RCLCPP_DEBUG(node_->get_logger(),
               "Interface::Interface(): started");

  position_last_ = 0.0;
  velocity_last_ = 0.0;
  velocity_last_position_ = 0.0;
  velocity_last_time_ = std::chrono::high_resolution_clock::now();

  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("encoder_prefix", std::string("/encoder/") + std::string(node_->get_name()));
  node->get_parameter("encoder_prefix", interface_prefix_);
  node->declare_parameter("encoder_readings_per_second", 100);
  node->get_parameter("encoder_readings_per_second", param_readings_);

  auto prefix = get_prefix_();

  position = node->create_publisher<std_msgs::msg::Float32>(
      prefix + "/position", 10);
  velocity = node->create_publisher<std_msgs::msg::Float32>(
      prefix + "/velocity", 10);

  position_get = node_->create_service<encoder::srv::PositionGet>(
      prefix + "/position",
      std::bind(&Interface::position_get_handler_, this,
                std::placeholders::_1, std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  velocity_get = node_->create_service<encoder::srv::VelocityGet>(
      prefix + "/velocity",
      std::bind(&Interface::velocity_get_handler_, this,
                std::placeholders::_1, std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  period_ = std::chrono::seconds(1) / param_readings_.as_double();
  thread_ = std::shared_ptr<std::thread>(new std::thread(&Interface::run_, this));

  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): ended");
}

Interface::~Interface() {
  stop_();
}

void Interface::stop_() {
  do_stop_ = true;
  thread_->join();
}

void Interface::run_() {
  while (!do_stop_) {
    std::this_thread::sleep_for(period_);
    readings_mutex_.lock();
    get_current_position_();
    get_current_velocity_();
    position->publish(std_msgs::msg::Float32().set__data(position_last_));
    velocity->publish(std_msgs::msg::Float32().set__data(velocity_last_));
    readings_mutex_.unlock();
  }
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() >0 && prefix[prefix.length()-1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

void Interface::position_get_handler_(
    const std::shared_ptr<encoder::srv::PositionGet::Request> request,
    std::shared_ptr<encoder::srv::PositionGet::Response> response) {
  (void)request;
  get_current_position_();
  response->position = position_last_;
  response->exception_code = 0;
}

void Interface::velocity_get_handler_(
    const std::shared_ptr<encoder::srv::VelocityGet::Request> request,
    std::shared_ptr<encoder::srv::VelocityGet::Response> response){
  (void)request;
  get_current_velocity_();
  response->velocity = velocity_last_;
  response->exception_code = 0;
}

void Interface::get_current_velocity_() {
  auto now = std::chrono::high_resolution_clock::now();
  auto time_delta = std::chrono::duration_cast<
                      std::chrono::duration<double>>(now - velocity_last_time_);
  auto position_delta = position_last_ - velocity_last_position_;
  if (param_overflow_.as_bool() && ::abs(position_delta) > 0.5) {
    // If this is a single turn encoder then,
    // for the sake of velocity calculation,
    // we need to compensate for the integer overrun (underrun).
    if (position_delta > 0) {
      position_delta -= 1.0;
    } else {
      position_delta += 1.0;
    }
  }

  velocity_last_time_ = now;
  velocity_last_position_ = position_last_;

  velocity_last_ = position_delta / time_delta.count();
}

}  // namespace encoder
