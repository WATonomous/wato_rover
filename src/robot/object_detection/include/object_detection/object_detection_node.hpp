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

#ifndef PACKAGE_NAME_NODE_HPP_
#define PACKAGE_NAME_NODE_HPP_

#include "object_detection/object_detection_core.hpp"
#include "rclcpp/rclcpp.hpp"

class Object_detectionNode : public rclcpp::Node
{
public:
  Object_detectionNode();

private:
  robot::Object_detectionCore object_detection_;
};

#endif
