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
    : node_{node}, position_last_{0.0}, velocity_last_{0.0} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto encoder_prefix = default_encoder_prefix;
  if (encoder_prefix == "") {
    encoder_prefix = "/encoder/" + std::string(node_->get_name());
  }

  int index = 0;
  std::string parameter_name;
  do {
    parameter_name = "encoder_prefix";
    if (index++ != 0) {
      parameter_name += "_" + std::to_string(index);
    }
  } while (node->has_parameter(parameter_name));

  node->declare_parameter(parameter_name, encoder_prefix);
  node->get_parameter(parameter_name, interface_prefix_);
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
