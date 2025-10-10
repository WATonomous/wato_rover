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

#include "producer_core.hpp"

#include <chrono>
#include <cmath>

namespace samples
{

ProducerCore::ProducerCore(float x, float y, float z)
: pos_x_(x)
, pos_y_(y)
, pos_z_(z)
, velocity_(0)
{}

void ProducerCore::update_velocity(int velocity)
{
  velocity_ = velocity;
}

void ProducerCore::update_position(double pos_x, double pos_y, double pos_z)
{
  pos_x_ = pos_x;
  pos_y_ = pos_y;
  pos_z_ = pos_z;
}

void ProducerCore::update_coordinates()
{
  pos_x_ += velocity_ / sqrt(3);
  pos_y_ += velocity_ / sqrt(3);
  pos_z_ += velocity_ / sqrt(3);
}

void ProducerCore::serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const
{
  msg.data = "x:" + std::to_string(pos_x_) + ";y:" + std::to_string(pos_y_) + ";z:" + std::to_string(pos_z_) + ";";
  msg.valid = true;
}

}  // namespace samples
