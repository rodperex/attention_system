// Copyright 2024 Intelligent Robotics Lab
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

namespace attention_system
{

using std::placeholders::_1;

BaseController::BaseController()
: CascadeLifecycleNode("attention_system_base"),
  pid_params_{0.0, 1.0, 0.0, 0.3},
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  declare_parameter("frame_id", frame_id_);
  declare_parameter("pid_min_ref", pid_params_[0]);
  declare_parameter("pid_max_ref", pid_params_[1]);
  declare_parameter("pid_min_output", pid_params_[2]);
  declare_parameter("pid_max_output", pid_params_[3]);

  get_parameter("frame_id", frame_id_);
  get_parameter("pid_min_ref", pid_params_[0]);
  get_parameter("pid_max_ref", pid_params_[1]);
  get_parameter("pid_min_output", pid_params_[2]);
  get_parameter("pid_max_output", pid_params_[3]);

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/out_vel", 10);

  pid_.set_pid(pid_params_[0], pid_params_[1], pid_params_[2], pid_params_[3]);

}

void
BaseController::control_cycle()
{
  RCLCPP_DEBUG(get_logger(), "Control cycle");


  geometry_msgs::msg::TransformStamped base2target_msg;
  tf2::Stamped<tf2::Transform> base2target;
  
  try {
    base2target_msg = tf_buffer_.lookupTransform("base_footprint", frame_id_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Could not transform %s to base_footprint: %s", frame_id_.c_str(), ex.what());
    return;
  }
  tf2::fromMsg(base2target_msg, base2target);
  
  double yaw = std::atan2(base2target.getOrigin().y(), base2target.getOrigin().x());
  yaw = yaw * 180.0 / M_PI;

  double command = pid_.get_output(yaw);

  RCLCPP_INFO(get_logger(), "Target at %.2fº. Attention command: %.2f rad/s", yaw, command);

  geometry_msgs::msg::Twist command_msg;
  command_msg.angular.z = command;
  vel_pub_->publish(command_msg);

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "BaseController on_activate");

  vel_pub_->on_activate();

  timer_ =
    create_wall_timer(50ms, std::bind(&BaseController::control_cycle, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "BaseController on_deactivate");

  timer_ = nullptr;
  vel_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace attention_system
