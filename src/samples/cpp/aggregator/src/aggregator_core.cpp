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

#include "aggregator_core.hpp"

#include <algorithm>

namespace samples
{

AggregatorCore::AggregatorCore(int64_t timestamp)
: raw_msg_count_(0)
, filtered_msg_count_(0)
, start_(timestamp)
, latest_raw_time_(-1)
, latest_filtered_time_(-1)
{}

double AggregatorCore::raw_frequency() const
{
  if (latest_raw_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(raw_msg_count_) / (latest_raw_time_ - start_);
}

double AggregatorCore::filtered_frequency() const
{
  if (latest_filtered_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(filtered_msg_count_) / (latest_filtered_time_ - start_);
}

void AggregatorCore::add_raw_msg(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  latest_raw_time_ = std::max(static_cast<int64_t>(msg->timestamp), latest_raw_time_);
  raw_msg_count_++;
}

void AggregatorCore::add_filtered_msg(const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  for (auto filtered_msg : msg->packets) {
    latest_filtered_time_ = std::max(static_cast<int64_t>(filtered_msg.timestamp), latest_filtered_time_);
  }
  filtered_msg_count_++;
}

}  // namespace samples
