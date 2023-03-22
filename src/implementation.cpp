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

Implementation::Implementation(rclcpp::Node *node)
        : Interface(node),
          velocity_last_position_{0.0},
          do_stop_{false} {
  velocity_last_time_ = std::chrono::high_resolution_clock::now();

  if (!node->has_parameter("encoder_readings_per_second")) {
    node->declare_parameter("encoder_readings_per_second", 0.0);
  }
  node->get_parameter("encoder_readings_per_second", param_readings_);

  if (!node->has_parameter("encoder_overflow")) {
    node->declare_parameter("encoder_overflow", false);
  }
  node->get_parameter("encoder_overflow", param_overflow_);
}

void Implementation::init_encoder() {
  auto prefix = get_prefix_();

  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

  if (has_position()) {
    topic_position_ = node_->create_publisher<std_msgs::msg::Float64>(
        prefix + REMOTE_ENCODER_TOPIC_POSITION, qos);
    srv_position_get_ = node_->create_service<remote_encoder::srv::PositionGet>(
        prefix + REMOTE_ENCODER_SERVICE_POSITION_GET,
        std::bind(&Implementation::position_get_handler_, this,
                  std::placeholders::_1, std::placeholders::_2),
        ::rmw_qos_profile_default, callback_group_);
  }

  // If the encoder does not have velocity features then this module approximates it anyway
  topic_velocity_ = node_->create_publisher<std_msgs::msg::Float64>(
      prefix + REMOTE_ENCODER_TOPIC_VELOCITY, qos);
  srv_velocity_get_ = node_->create_service<remote_encoder::srv::VelocityGet>(
      prefix + REMOTE_ENCODER_SERVICE_VELOCITY_GET,
      std::bind(&Implementation::velocity_get_handler_, this,
                std::placeholders::_1, std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  if ((has_position() || has_velocity()) && param_readings_.as_double() > 0.0) {
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
  double value = position_last_;
  readings_mutex_.unlock();

  topic_position_->publish(std_msgs::msg::Float64().set__data(value));

  return value;
}

double Implementation::velocity_get() {
  readings_mutex_.lock();
  velocity_get_real_();
  double value = velocity_last_;
  readings_mutex_.unlock();

  topic_velocity_->publish(std_msgs::msg::Float64().set__data(value));

  return value;
}

Implementation::~Implementation() { stop_(); }

void Implementation::stop_() {
  do_stop_ = true;
  thread_->join();
}

void Implementation::run_() {
  bool do_position = has_position();

  while (!do_stop_) {
    std::this_thread::sleep_for(period_);
    readings_mutex_.lock();
    if (do_position) {
      position_get_real_();
    }
    auto position_last = position_last_;
    velocity_get_real_();
    auto velocity_last = velocity_last_;
    readings_mutex_.unlock();

    if (do_position) {
      topic_position_->publish(std_msgs::msg::Float64().set__data(position_last));
    }
    topic_velocity_->publish(std_msgs::msg::Float64().set__data(velocity_last));
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
