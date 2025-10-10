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

#include "gtest/gtest.h"
#include "producer_core.hpp"

TEST(ProducerTest, ValidSerialization)
{
  auto producer = samples::ProducerCore(11, -12, 0);
  auto msg = sample_msgs::msg::Unfiltered();
  producer.serialize_coordinates(msg);

  EXPECT_TRUE(msg.valid);
  EXPECT_EQ(msg.data, "x:11.000000;y:-12.000000;z:0.000000;");
}

TEST(ProducerTest, NoCoordinateUpdate)
{
  auto producer = samples::ProducerCore();
  producer.update_coordinates();

  auto msg = sample_msgs::msg::Unfiltered();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:0.000000;y:0.000000;z:0.000000;");
}

TEST(ProducerTest, MultipleCoordinateUpdate)
{
  auto producer = samples::ProducerCore();
  auto msg = sample_msgs::msg::Unfiltered();

  producer.update_velocity(1);
  producer.update_coordinates();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:0.577350;y:0.577350;z:0.577350;");

  producer.update_velocity(-4);
  producer.update_coordinates();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:-1.732051;y:-1.732051;z:-1.732051;");
}
