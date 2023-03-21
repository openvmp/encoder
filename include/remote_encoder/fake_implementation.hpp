/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ENCODER_FAKE_IMPLEMENTATION_H
#define OPENVMP_ENCODER_FAKE_IMPLEMENTATION_H

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/implementation.hpp"

namespace remote_encoder {

class FakeImplementation : public Implementation {
 public:
  FakeImplementation(rclcpp::Node *node);
  virtual ~FakeImplementation();

 protected:
  // subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      subscription_actuator_position_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      subscription_actuator_velocity_;

  virtual bool has_position() override { return true; }
  virtual bool has_velocity() override { return true; }
  virtual void position_get_real_() override;
  virtual void velocity_get_real_() override;

 private:
  std::shared_ptr<std::thread> fake_thread_;
  volatile bool fake_do_stop_;
  std::chrono::duration<double, std::ratio<1L>> fake_period_;

  void fake_run_();
  void fake_stop_();

  // double actuator_velocity_actual_;
  double actuator_position_last_;
  double actuator_velocity_last_;

  void sub_position_handler_(const std_msgs::msg::Float64::SharedPtr);
  void sub_velocity_handler_(const std_msgs::msg::Float64::SharedPtr);
};

}  // namespace remote_encoder

#endif  // OPENVMP_ENCODER_FAKE_IMPLEMENTATION_H
