/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ENCODER_FAKE_NODE_H
#define OPENVMP_ENCODER_FAKE_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/fake_implementation.hpp"

namespace remote_encoder {

class FakeNode : public rclcpp::Node {
 public:
  FakeNode();

  std::shared_ptr<FakeImplementation> intf_;
};

}  // namespace remote_encoder

#endif  // OPENVMP_ENCODER_FAKE_NODE_H
