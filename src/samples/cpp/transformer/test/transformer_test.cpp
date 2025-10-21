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

#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "transformer/transformer_core.hpp"

/**
 * When writing a large number of tests it is desirable to wrap any common
 * setup or cleanup logic in a test fixture. This improves readability and reduces
 * the amount of boilerplate code in each test. For more information checkout
 * https://google.github.io/googletest/primer.html#same-data-multiple-tests
 */
class TransformerFixtureTest : public ::testing::Test
{
public:
  void SetUp(int enqueue_count)
  {
    auto filtered = sample_msgs::msg::Filtered();
    for (int i = 0; i < enqueue_count; i++) {
      transformer.enqueue_message(filtered);
    }
  }

protected:
  samples::TransformerCore transformer;
};

/**
 * Parameterized tests let us test the same logic with different parameters without
 * having to write boilerplate for multiple tests. For more information checkout
 * https://google.github.io/googletest/advanced.html#value-parameterized-tests
 */
class TransformerParameterizedTest : public ::testing::TestWithParam<std::tuple<const char *, bool>>
{
protected:
  samples::TransformerCore transformer;
};

TEST(TransformerTest, FilterInvalidField)
{
  auto unfiltered = std::make_shared<sample_msgs::msg::Unfiltered>();
  unfiltered->valid = false;

  auto transformer = samples::TransformerCore();
  bool valid = transformer.validate_message(unfiltered);
  EXPECT_FALSE(valid);
}

TEST_F(TransformerFixtureTest, BufferCapacity)
{
  SetUp(samples::TransformerCore::BUFFER_CAPACITY - 1);

  // Place last message that fits in buffer
  auto filtered = sample_msgs::msg::Filtered();
  bool full = transformer.enqueue_message(filtered);
  EXPECT_TRUE(full);
  int size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 10);

  // Attempting to enqueue message to full buffer should fail
  full = transformer.enqueue_message(filtered);
  EXPECT_TRUE(full);
  size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 10);
}

TEST_F(TransformerFixtureTest, ClearBuffer)
{
  // Place 3 messages in buffer
  SetUp(3);

  int size = transformer.buffer_messages().size();
  EXPECT_GT(size, 0);

  transformer.clear_buffer();
  size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 0);
}

TEST_P(TransformerParameterizedTest, SerializationValidation)
{
  auto [serialized_field, valid] = GetParam();
  auto filtered = sample_msgs::msg::Filtered();
  auto unfiltered = std::make_shared<sample_msgs::msg::Unfiltered>();
  unfiltered->data = serialized_field;
  EXPECT_EQ(transformer.deserialize_coordinate(unfiltered, filtered), valid);
}

// Parameterized testing lets us validate all edge cases for serialization
// using one test case.
INSTANTIATE_TEST_CASE_P(
  Serialization,
  TransformerParameterizedTest,
  ::testing::Values(
    std::make_tuple("x:1;y:2;z:3", false),
    std::make_tuple("z:1;y:2;x:3;", false),
    std::make_tuple("x:1,y:2,z:3", false),
    std::make_tuple("x:3;", false),
    std::make_tuple("x:3;y:2;z:3;", true),
    std::make_tuple("x:3;y:22; z:11;", true)));
