/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_encoder/interface.hpp"

#include <functional>

namespace remote_encoder {

Interface::Interface(rclcpp::Node *node,
                     const std::string &default_encoder_prefix)
    : node_{node} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto encoder_prefix = default_encoder_prefix;
  if (encoder_prefix == "") {
    encoder_prefix = "/encoder/" + std::string(node_->get_name());
  }
  node->declare_parameter("encoder_prefix", encoder_prefix);
  node->get_parameter("encoder_prefix", interface_prefix_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace remote_encoder
