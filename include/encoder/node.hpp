/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ENCODER_NODE_H
#define OPENVMP_ENCODER_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace encoder {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  // node parameters
  rclcpp::Parameter prefix_;
};

}  // namespace encoder

#endif  // OPENVMP_ENCODER_NODE_H
