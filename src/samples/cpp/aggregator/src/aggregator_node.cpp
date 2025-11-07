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

#include "aggregator/aggregator_node.hpp"

#include <chrono>
#include <memory>

AggregatorNode::AggregatorNode()
: Node("aggregator")
, aggregator_(samples::AggregatorCore(
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()))
{
  raw_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "/unfiltered_topic",
    ADVERTISING_FREQ,
    std::bind(&AggregatorNode::unfiltered_callback, this, std::placeholders::_1));
  filtered_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "/filtered_topic", ADVERTISING_FREQ, std::bind(&AggregatorNode::filtered_callback, this, std::placeholders::_1));
}

void AggregatorNode::unfiltered_callback(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  aggregator_.add_raw_msg(msg);
  RCLCPP_INFO(this->get_logger(), "Raw Frequency(msg/s): %f", aggregator_.raw_frequency() * 1000);
}

void AggregatorNode::filtered_callback(const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  aggregator_.add_filtered_msg(msg);
  RCLCPP_INFO(this->get_logger(), "Filtered Frequency(msg/s): %f", aggregator_.filtered_frequency() * 1000);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AggregatorNode>());
  rclcpp::shutdown();
  return 0;
}
