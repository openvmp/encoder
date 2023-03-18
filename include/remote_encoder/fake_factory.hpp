/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_ENCODER_FAKE_FACTORY_H
#define OPENVMP_REMOTE_ENCODER_FAKE_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/interface.hpp"

namespace remote_encoder {

class FakeFactory {
 public:
  static std::shared_ptr<Interface> New(
      rclcpp::Node *node, const std::string &default_encoder_prefix = "");
};

}  // namespace remote_encoder

#endif  // OPENVMP_REMOTE_ENCODER_FAKE_FACTORY_H