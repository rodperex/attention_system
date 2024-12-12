// Copyright 2024 Rodrigo Pérez-Rodríguez
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

#include "attention_system/BaseController.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;


  auto attention_node = std::make_shared<attention_system::BaseController>();
//   attention_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//   attention_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);


  exec.add_node(attention_node->get_node_base_interface());

  while (rclcpp::ok()) {
    exec.spin_some();
  }

  rclcpp::shutdown();

  return 0;
}
