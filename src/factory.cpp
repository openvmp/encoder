/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-05
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_encoder/factory.hpp"

#include <exception>

#include "remote_encoder/interface_remote.hpp"

namespace remote_encoder {

std::shared_ptr<Interface> Factory::New(
    rclcpp::Node *node, const std::string &default_encoder_prefix) {
  rclcpp::Parameter is_remote;
  node->declare_parameter("encoder_is_remote", true);
  node->get_parameter("encoder_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node, default_encoder_prefix);
  } else {
    throw std::invalid_argument(
        "Link with the actual driver or set encoder_is_remote");
  }
}

}  // namespace remote_encoder