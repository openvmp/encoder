/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "encoder/node.hpp"

namespace encoder {

Node::Node() : rclcpp::Node::Node("encoder_terminal") {
  this->declare_parameter("prefix", "/encoder/");
  this->get_parameter("prefix", prefix_);

  // TODO(clairbee): enumerate services here and print an error
  //                  or instantiate the UI, allow adhoc readings from encoders
}

}  // namespace encoder
