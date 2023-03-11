/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-05
 *
 * Licensed under Apache License, Version 2.0.
 */
#ifndef OPENVMP_REMOTE_ENCODER_INTERFACE_REMOTE_H
#define OPENVMP_REMOTE_ENCODER_INTERFACE_REMOTE_H

#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/interface.hpp"
#include "std_msgs/msg/float64.hpp"

namespace remote_encoder {

class RemoteInterface final : public Interface {
 public:
  RemoteInterface(rclcpp::Node *node,
                  const std::string &default_encoder_prefix = "");
  virtual ~RemoteInterface() {}

  virtual bool has_position() override;
  virtual bool has_velocity() override;
  virtual double position_get() override;
  virtual double velocity_get() override;

  void sub_position_handler_(const std_msgs::msg::Float64::SharedPtr);
  void sub_velocity_handler_(const std_msgs::msg::Float64::SharedPtr);

 private:
  bool has_position_;
  bool has_velocity_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_position_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_velocity_;

  rclcpp::Client<remote_encoder::srv::PositionGet>::SharedPtr
      clnt_position_get_;
  rclcpp::Client<remote_encoder::srv::VelocityGet>::SharedPtr
      clnt_velocity_get_;

  rclcpp::Client<remote_encoder::srv::PositionGet>::SharedPtr
  get_clnt_position_get_();
  rclcpp::Client<remote_encoder::srv::VelocityGet>::SharedPtr
  get_clnt_velocity_get_();
};

}  // namespace remote_encoder

#endif  // OPENVMP_REMOTE_ENCODER_INTERFACE_REMOTE_H
