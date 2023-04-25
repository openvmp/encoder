/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_ENCODER_IMPLEMENTATION_H
#define OPENVMP_REMOTE_ENCODER_IMPLEMENTATION_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/interface.hpp"
#include "remote_encoder/srv/velocity_get.hpp"
#include "std_msgs/msg/float64.hpp"

namespace remote_encoder {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node);
  virtual ~Implementation();

  virtual double velocity_get() override final;
  virtual double position_get() override final;

 protected:
  rclcpp::Parameter param_readings_;
  rclcpp::Parameter param_overflow_;

  double velocity_last_position_;
  std::chrono::high_resolution_clock::time_point velocity_last_time_;

  void init_encoder_();

  virtual void position_get_real_() = 0;
  virtual void velocity_get_real_();

 private:
  std::shared_ptr<std::thread> thread_;
  volatile bool do_stop_;
  std::chrono::duration<double, std::ratio<1L>> period_;

  void run_();
  void stop_();

  // topics
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr topic_position_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr topic_velocity_;

  // services
  rclcpp::Service<remote_encoder::srv::PositionGet>::SharedPtr
      srv_position_get_;
  rclcpp::Service<remote_encoder::srv::VelocityGet>::SharedPtr
      srv_velocity_get_;

  rclcpp::FutureReturnCode position_get_handler_(
      const std::shared_ptr<remote_encoder::srv::PositionGet::Request> request,
      std::shared_ptr<remote_encoder::srv::PositionGet::Response> response);
  rclcpp::FutureReturnCode velocity_get_handler_(
      const std::shared_ptr<remote_encoder::srv::VelocityGet::Request> request,
      std::shared_ptr<remote_encoder::srv::VelocityGet::Response> response);
};

}  // namespace remote_encoder

#endif  // OPENVMP_REMOTE_ENCODER_IMPLEMENTATION_H
