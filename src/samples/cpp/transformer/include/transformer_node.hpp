// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRANSFORMER_NODE_HPP_
#define TRANSFORMER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sample_msgs/msg/filtered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/unfiltered.hpp"
#include "transformer_core.hpp"

/**
 * Implementation of a ROS2 node that converts unfiltered messages to filtered_array
 * messages.
 *
 * Listens to the "unfiltered" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class TransformerNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Transformer node constructor.
   */
  TransformerNode();

private:
  /**
   * A ROS2 subscription node callback used to process raw data from the
   * "unfiltered" topic and publish to the "filtered" topic.
   *
   * @param msg a raw message from the "unfiltered" topic
   */
  void unfiltered_callback(const sample_msgs::msg::Unfiltered::SharedPtr msg);

  // ROS2 subscriber listening to the unfiltered topic.
  rclcpp::Subscription<sample_msgs::msg::Unfiltered>::SharedPtr raw_sub_;

  // ROS2 publisher sending processed messages to the filtered topic.
  rclcpp::Publisher<sample_msgs::msg::FilteredArray>::SharedPtr transform_pub_;

  // Object that handles data processing and validation.
  samples::TransformerCore transformer_;
};

#endif  // TRANSFORMER_NODE_HPP_
