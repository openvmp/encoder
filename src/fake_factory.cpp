/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_encoder/fake_factory.hpp"

#include <exception>

#include "remote_encoder/fake_implementation.hpp"
#include "remote_encoder/interface_remote.hpp"

namespace remote_encoder {

std::shared_ptr<Interface> FakeFactory::New(
    rclcpp::Node *node, const std::string &default_encoder_prefix) {
  rclcpp::Parameter use_remote;
  if (!node->has_parameter("use_remote")) {
    node->declare_parameter("use_remote", true);
  }
  node->get_parameter("use_remote", use_remote);

  rclcpp::Parameter is_remote;
  if (!node->has_parameter("encoder_is_remote")) {
    node->declare_parameter("encoder_is_remote", use_remote.as_bool());
  }
  node->get_parameter("encoder_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node, default_encoder_prefix);
  } else {
    return std::make_shared<FakeImplementation>(node);
  }
}

}  // namespace remote_encoder
