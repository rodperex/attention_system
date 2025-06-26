// Copyright 2021 Intelligent Robotics Lab
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

#ifndef ATTENTION_SYSTEM__HPP_
#define ATTENTION_SYSTEM__HPP_

#include <memory>
#include <algorithm>
#include <utility>

#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp/rclcpp.hpp"

#include "attention_system/PID.hpp"

// #include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"

#include "attention_msgs/srv/activate_attention.hpp"
#include "std_srvs/srv/trigger.hpp"

class PID;
namespace attention_system
{


using namespace std::chrono_literals;
class BaseController : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  BaseController();

  void control_cycle();

private:
  void
    target_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state);
  void
  handle_activation_service(const std::shared_ptr<attention_msgs::srv::ActivateAttention::Request> request, 
      std::shared_ptr<attention_msgs::srv::ActivateAttention::Response> response);
  void
  handle_deactivation_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  rclcpp::TimerBase::SharedPtr timer_;

  double pid_params_[4];

  PID pid_;

  std::string frame_id_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  rclcpp::Service<attention_msgs::srv::ActivateAttention>::SharedPtr activate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deactivate_srv_;

  double max_time_no_tf_;
  rclcpp::Time last_tf_time_;

};

}  // namespace attention_system

#endif  // BC__ATTENTION_SYSTEM__HPP_
