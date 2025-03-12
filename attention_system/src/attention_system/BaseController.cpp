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
using std::placeholders::_2;

BaseController::BaseController()
: CascadeLifecycleNode("attention_system_base"),
  pid_params_{0.0, 1.0, 0.0, 0.3},
  frame_id_(""),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  double kp, ki, kd;
  
  declare_parameter("pid_min_ref", pid_params_[0]);
  declare_parameter("pid_max_ref", pid_params_[1]);
  declare_parameter("pid_min_output", pid_params_[2]);
  declare_parameter("pid_max_output", pid_params_[3]);
  declare_parameter("pid_kp", 0.41);
  declare_parameter("pid_ki", 0.06);
  declare_parameter("pid_kd", 0.53);


  get_parameter("pid_min_ref", pid_params_[0]);
  get_parameter("pid_max_ref", pid_params_[1]);
  get_parameter("pid_min_output", pid_params_[2]);
  get_parameter("pid_max_output", pid_params_[3]);
  get_parameter("pid_kp", kp);
  get_parameter("pid_ki", ki);
  get_parameter("pid_kd", kd);
  get_parameter("max_time_no_tf", max_time_no_tf_);

  


  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/out_vel", 10);

  target_sub_ = create_subscription<std_msgs::msg::String>(
    "attention_system", 10, std::bind(&BaseController::target_callback, this, _1));

  pid_.set_pid(pid_params_[0], pid_params_[1], pid_params_[2], pid_params_[3]);
  pid_.set_constants(kp, ki, kd);

}

void
BaseController::target_callback(const std_msgs::msg::String::SharedPtr msg)
{
  frame_id_ = msg->data;

  if (frame_id_ == "") {
    RCLCPP_INFO(get_logger(), "Deactivating attention system");
  } else {
    RCLCPP_DEBUG(get_logger(), "Attending to frame %s", frame_id_.c_str());
  }
}

void
BaseController::control_cycle()
{
  RCLCPP_DEBUG(get_logger(), "Control cycle");

  if (frame_id_ == "") {
    return;
  }
  geometry_msgs::msg::TransformStamped base2target_msg;
  tf2::Stamped<tf2::Transform> base2target;
  
  try {
    base2target_msg = tf_buffer_.lookupTransform("base_footprint", frame_id_, tf2::TimePointZero);
    last_tf_time_ = now();
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Could not transform %s to base_footprint: %s", frame_id_.c_str(), ex.what());

    if ((now() - last_tf_time_).seconds() > max_time_no_tf_) {
      RCLCPP_ERROR_ONCE(get_logger(), "No TF for more than %.2f seconds. Deactivating attention system", max_time_no_tf_);
      frame_id_ = "";
    }

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

void
BaseController::handle_activation_service(
  const std::shared_ptr<attention_msgs::srv::ActivateAttention::Request> request,
  std::shared_ptr<attention_msgs::srv::ActivateAttention::Response> response)
{
  RCLCPP_INFO(get_logger(), "Activate Attention service called");
  frame_id_ = request->frame_id;
  timer_ =
    create_wall_timer(50ms, std::bind(&BaseController::control_cycle, this));
  response->success = true;
}

void
BaseController::handle_deactivation_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  RCLCPP_INFO(get_logger(), "Deactivate Attention service called");
  timer_ = nullptr;
  response->success = true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Attention system node activated");

  activate_srv_ = create_service<attention_msgs::srv::ActivateAttention>(
    "activate_attention", std::bind(&BaseController::handle_activation_service, this, _1, _2));
  
  deactivate_srv_ = create_service<std_srvs::srv::Trigger>(
    "deactivate_attention", std::bind(&BaseController::handle_deactivation_service, this, _1, _2));

  vel_pub_->on_activate();

  timer_ =
    create_wall_timer(50ms, std::bind(&BaseController::control_cycle, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Attention system node deactivated");

  timer_ = nullptr;
  vel_pub_->on_deactivate();

  activate_srv_.reset();
  deactivate_srv_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace attention_system
