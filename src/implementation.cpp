/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-05
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_encoder/implementation.hpp"

namespace remote_encoder {

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

  position_last_ = 0.0;
  velocity_last_ = 0.0;
  velocity_last_position_ = 0.0;
  velocity_last_time_ = std::chrono::high_resolution_clock::now();

  topic_position_ = node->create_publisher<std_msgs::msg::Float64>(
      prefix + REMOTE_ENCODER_TOPIC_POSITION,
      rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  topic_velocity_ = node->create_publisher<std_msgs::msg::Float64>(
      prefix + REMOTE_ENCODER_TOPIC_VELOCITY,
      rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  srv_position_get_ = node_->create_service<remote_encoder::srv::PositionGet>(
      prefix + REMOTE_ENCODER_SERVICE_POSITION_GET,
      std::bind(&Implementation::position_get_handler_, this,
                std::placeholders::_1, std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  srv_velocity_get_ = node_->create_service<remote_encoder::srv::VelocityGet>(
      prefix + REMOTE_ENCODER_SERVICE_VELOCITY_GET,
      std::bind(&Implementation::velocity_get_handler_, this,
                std::placeholders::_1, std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  node->declare_parameter("encoder_readings_per_second", 0.0);
  node->get_parameter("encoder_readings_per_second", param_readings_);

  if (param_readings_.as_double() > 0.0) {
    period_ = std::chrono::seconds(1) / param_readings_.as_double();
    thread_ = std::shared_ptr<std::thread>(
        new std::thread(&Implementation::run_, this));
  }
}

rclcpp::FutureReturnCode Implementation::position_get_handler_(
    const std::shared_ptr<remote_encoder::srv::PositionGet::Request> request,
    std::shared_ptr<remote_encoder::srv::PositionGet::Response> response) {
  (void)request;
  position_get_real_();
  response->position = position_last_;
  response->exception_code = 0;
  return rclcpp::FutureReturnCode::SUCCESS;
}

rclcpp::FutureReturnCode Implementation::velocity_get_handler_(
    const std::shared_ptr<remote_encoder::srv::VelocityGet::Request> request,
    std::shared_ptr<remote_encoder::srv::VelocityGet::Response> response) {
  (void)request;
  velocity_get_real_();
  response->velocity = velocity_last_;
  response->exception_code = 0;
  return rclcpp::FutureReturnCode::SUCCESS;
}

double Implementation::position_get() {
  readings_mutex_.lock();

  position_get_real_();
  topic_position_->publish(std_msgs::msg::Float64().set__data(position_last_));

  readings_mutex_.unlock();
}

double Implementation::velocity_get() {
  readings_mutex_.lock();

  velocity_get_real_();
  topic_velocity_->publish(std_msgs::msg::Float64().set__data(velocity_last_));

  readings_mutex_.unlock();
}

Implementation::~Implementation() { stop_(); }

void Implementation::stop_() {
  do_stop_ = true;
  thread_->join();
}

void Implementation::run_() {
  while (!do_stop_) {
    std::this_thread::sleep_for(period_);
    readings_mutex_.lock();
    position_get_real_();
    velocity_get_real_();
    topic_position_->publish(
        std_msgs::msg::Float64().set__data(position_last_));
    topic_velocity_->publish(
        std_msgs::msg::Float64().set__data(velocity_last_));
    readings_mutex_.unlock();
  }
}

void Implementation::velocity_get_real_() {
  auto now = std::chrono::high_resolution_clock::now();
  auto time_delta = std::chrono::duration_cast<std::chrono::duration<double>>(
      now - velocity_last_time_);
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

}  // namespace remote_encoder