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

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/srv/position_get.hpp"
#include "remote_encoder/srv/velocity_get.hpp"

#define REMOTE_ENCODER_TOPIC_POSITION "/position"
#define REMOTE_ENCODER_TOPIC_VELOCITY "/velocity"
#define REMOTE_ENCODER_SERVICE_POSITION_GET "/get_position"
#define REMOTE_ENCODER_SERVICE_VELOCITY_GET "/get_velocity"

namespace remote_encoder {

class Interface {
 public:
  Interface(rclcpp::Node *node,
            const std::string &default_actuator_prefix = "");
  virtual ~Interface() {}

  virtual bool has_position() { return false; }
  virtual bool has_velocity() { return false; }
  virtual double position_get() = 0;
  virtual double velocity_get() = 0;

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::mutex readings_mutex_;
  double position_last_;
  double velocity_last_;

  std::string get_prefix_();

 private:
  rclcpp::Parameter interface_prefix_;
};

}  // namespace remote_encoder

#endif  // OPENVMP_ENCODER_INTERFACE_H
