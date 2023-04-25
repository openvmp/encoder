/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_encoder/fake_implementation.hpp"

#include <algorithm>
#include <functional>

#include "remote_actuator/interface.hpp"

namespace remote_encoder {

FakeImplementation::FakeImplementation(rclcpp::Node *node)
    : Implementation(node),
      fake_do_stop_{false},
      // actuator_velocity_actual_{0.0},
      actuator_position_last_{0.0},
      actuator_velocity_last_{0.0} {
  init_encoder_();

  auto actuator_prefix = get_prefix_();
  std::string from = "encoder";
  std::string to = "actuator";
  size_t start_pos = 0;
  while ((start_pos = actuator_prefix.find(from, start_pos)) !=
         std::string::npos) {
    actuator_prefix.replace(start_pos, from.length(), to);
    start_pos +=
        to.length();  // Handles case where 'to' is a substring of 'from'
  }
  // 'actuator_prefix' now points at the corresponding actuator

  subscription_actuator_position_ =
      node_->create_subscription<std_msgs::msg::Float64>(
          actuator_prefix + REMOTE_ACTUATOR_TOPIC_POSITION, 10,
          std::bind(&FakeImplementation::sub_position_handler_, this,
                    std::placeholders::_1));

  subscription_actuator_velocity_ =
      node_->create_subscription<std_msgs::msg::Float64>(
          actuator_prefix + REMOTE_ACTUATOR_TOPIC_VELOCITY, 10,
          std::bind(&FakeImplementation::sub_velocity_handler_, this,
                    std::placeholders::_1));

  auto readings_per_second = param_readings_.as_double();
  if (readings_per_second > 0.0) {
    fake_period_ = std::chrono::seconds(1) / readings_per_second;
    fake_thread_ = std::shared_ptr<std::thread>(
        new std::thread(&FakeImplementation::fake_run_, this));
  }
}

FakeImplementation::~FakeImplementation() { fake_stop_(); }

void FakeImplementation::fake_stop_() {
  fake_do_stop_ = true;
  fake_thread_->join();
}

void FakeImplementation::fake_run_() {
  auto readings_per_second = param_readings_.as_double();

  while (!fake_do_stop_) {
    std::this_thread::sleep_for(fake_period_);

    readings_mutex_.lock();
    actuator_position_last_ += actuator_velocity_last_ / readings_per_second;
    // actuator_position_last_ += actuator_velocity_actual_ /
    // readings_per_second; auto delta = (actuator_velocity_last_ -
    // actuator_velocity_actual_); auto step = delta / 2.0; if (::abs(step) <
    // 0.02) step = delta; actuator_velocity_actual_ += step;
    readings_mutex_.unlock();
  }
}

void FakeImplementation::sub_position_handler_(
    const std_msgs::msg::Float64::SharedPtr msg) {
  readings_mutex_.lock();
  actuator_position_last_ = msg->data;
  readings_mutex_.unlock();
  // FIXME(clairbee): fake_do_stop_()? no need to estimate
}

void FakeImplementation::sub_velocity_handler_(
    const std_msgs::msg::Float64::SharedPtr msg) {
  readings_mutex_.lock();
  actuator_velocity_last_ = msg->data;
  readings_mutex_.unlock();
}

void FakeImplementation::position_get_real_() {
  // already locked
  position_last_ = actuator_position_last_;
}
void FakeImplementation::velocity_get_real_() {
  // already locked
  velocity_last_ = actuator_velocity_last_;
}

}  // namespace remote_encoder
