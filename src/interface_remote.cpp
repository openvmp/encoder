/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-05
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_encoder/interface_remote.hpp"

#include <functional>

namespace remote_encoder {

RemoteInterface::RemoteInterface(rclcpp::Node *node,
                                 const std::string &default_encoder_prefix)
    : Interface(node, default_encoder_prefix) {
  auto prefix = get_prefix_();
  auto callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  RCLCPP_DEBUG(
      node_->get_logger(),
      "remote_encoder::RemoteInterface::RemoteInterface(): Connecting to the "
      "remote interface: %s",
      prefix.c_str());

  RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
               prefix.c_str());
}

void RemoteInterface::sub_position_handler_(
    const std_msgs::msg::Float64::SharedPtr msg) {
  readings_mutex_.lock();
  position_last_ = msg->data;
  readings_mutex_.unlock();
}

void RemoteInterface::sub_velocity_handler_(
    const std_msgs::msg::Float64::SharedPtr msg) {
  readings_mutex_.lock();
  velocity_last_ = msg->data;
  readings_mutex_.unlock();
}

rclcpp::Client<remote_encoder::srv::PositionGet>::SharedPtr
RemoteInterface::get_clnt_position_get_() {
  if (!clnt_position_get_) {
    auto prefix = get_prefix_();

    clnt_position_get_ = node_->create_client<remote_encoder::srv::PositionGet>(
        prefix + REMOTE_ENCODER_SERVICE_POSITION_GET, ::rmw_qos_profile_default,
        callback_group_);

    has_position_ =
        clnt_velocity_get_->wait_for_service(std::chrono::milliseconds(100));
    if (has_position_) {
      sub_position_ = node_->create_subscription<std_msgs::msg::Float64>(
          prefix + REMOTE_ENCODER_TOPIC_POSITION, 1,
          std::bind(&RemoteInterface::sub_position_handler_, this,
                    std::placeholders::_1));
    }
  }
  return clnt_position_get_;
}

rclcpp::Client<remote_encoder::srv::VelocityGet>::SharedPtr
RemoteInterface::get_clnt_velocity_get_() {
  if (!clnt_velocity_get_) {
    auto prefix = get_prefix_();

    clnt_velocity_get_ = node_->create_client<remote_encoder::srv::VelocityGet>(
        prefix + REMOTE_ENCODER_SERVICE_VELOCITY_GET, ::rmw_qos_profile_default,
        callback_group_);

    has_velocity_ =
        clnt_velocity_get_->wait_for_service(std::chrono::milliseconds(100));
    if (has_velocity_) {
      sub_velocity_ = node_->create_subscription<std_msgs::msg::Float64>(
          prefix + REMOTE_ENCODER_TOPIC_VELOCITY, 1,
          std::bind(&RemoteInterface::sub_velocity_handler_, this,
                    std::placeholders::_1));
    }
  }
  return clnt_velocity_get_;
}

bool RemoteInterface::has_position() {
  (void)get_clnt_position_get_();
  return has_position_;
}

bool RemoteInterface::has_velocity() {
  (void)get_clnt_velocity_get_();
  return has_velocity_;
}

double RemoteInterface::position_get() {
  if (!has_position()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "RemoteInterface::position_get(): not connected");
    return 0.0;
  }

  // auto req = std::make_shared<remote_encoder::srv::PositionGet::Request>();
  // auto resp = std::make_shared<remote_encoder::srv::PositionGet::Response>();

  // auto f = clnt_position_get_->async_send_request(req);
  // f.wait();
  // RCLCPP_DEBUG(node_->get_logger(),
  //              "RemoteInterface::position_get(): response received");

  // position_last_ = resp->position;

  // Assuming the subscription worked fine
  return position_last_;
}

double RemoteInterface::velocity_get() {
  if (!has_velocity()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "RemoteInterface::velocity_get(): not connected");
    return 0.0;
  }

  // auto req = std::make_shared<remote_encoder::srv::VelocityGet::Request>();
  // auto resp = std::make_shared<remote_encoder::srv::VelocityGet::Response>();

  // auto f = clnt_velocity_get_->async_send_request(req);
  // f.wait();
  // RCLCPP_DEBUG(node_->get_logger(),
  //              "RemoteInterface::velocity_get(): response received");

  // velocity_last_ = resp->velocity;

  // Assuming the subscription worked fine
  return velocity_last_;
}

}  // namespace remote_encoder
