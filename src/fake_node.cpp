/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_encoder/fake_node.hpp"

namespace remote_encoder {

FakeNode::FakeNode()
    : rclcpp::Node(
          "encoder_fake",
          rclcpp::NodeOptions().parameter_overrides(
              std::vector<rclcpp::Parameter>{
                  rclcpp::Parameter("encoder_readings_per_second", 50.0),
              })) {
  intf_ = std::make_shared<FakeImplementation>(this);
}

}  // namespace remote_encoder
