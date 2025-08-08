// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <mutex>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <algorithm>

namespace franka_example_controllers {

bool CartesianVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  sub_vel_command_ = node_handle.subscribe(
    "vel_command",1 , &CartesianVelocityExampleController::cartesianvelocitycallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // for (size_t i = 0; i < q_start.size(); i++) {
    //   if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
    //     ROS_ERROR_STREAM(
    //         "CartesianVelocityExampleController: Robot is not in the expected starting position "
    //         "for running this example. Run `roslaunch franka_example_controllers "
    //         "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
    //         "first.");
    //     return false;
    //   }
    // }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  

  return true;
}

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  target_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  current_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  start_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void CartesianVelocityExampleController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;

  // double time_max = 4.0;
  // double v_max = 0.05;
  // double angle = M_PI / 4.0;
  // double cycle = std::floor(
  //     pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
  // double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
  // double v_x = std::cos(angle) * v;
  // double v_z = -std::sin(angle) * v;
  // std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
  
  std::array<double, 6> command;

  if (smoothing_active_) {
    double t = (ros::Time::now() - smoothing_start_time_).toSec();

    if (t >= smoothing_time_) {
      current_velocity_ = target_velocity_;
      smoothing_active_ = false;
    } else {
      double ratio = 0.5 * (1.0 - std::cos(M_PI * t / smoothing_time_));
      for (size_t i = 0; i < 6; ++i) {
        current_velocity_[i] = start_velocity_[i] + (target_velocity_[i] - start_velocity_[i]) * ratio;
      }
    }
  }

  // 将当前平滑速度发送给 Franka
  velocity_cartesian_handle_->setCommand(current_velocity_);

  
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityExampleController::cartesianvelocitycallback(const geometry_msgs::Twist::ConstPtr& msg){
  // v_x = msg->linear.x;
  // v_y = msg->linear.y;
  // v_z = msg->linear.z;
  // w_x = msg->angular.x;
  // w_y = msg->angular.y;
  // w_z = msg->angular.z;

  std::lock_guard<std::mutex> lock(command_mutex_);
  
  // 设置新的目标速度，限制最大值
  target_velocity_[0] = std::max(-v_max_, std::min(msg->linear.x, v_max_));
  target_velocity_[1] = std::max(-v_max_, std::min(msg->linear.y, v_max_));
  target_velocity_[2] = std::max(-v_max_, std::min(msg->linear.z, v_max_));
  target_velocity_[3] = std::max(-v_max_, std::min(msg->angular.x, v_max_));
  target_velocity_[4] = std::max(-v_max_, std::min(msg->angular.y, v_max_));
  target_velocity_[5] = std::max(-v_max_, std::min(msg->angular.z, v_max_));

  start_velocity_ = current_velocity_;
  smoothing_start_time_ = ros::Time::now();
  smoothing_active_ = true;

}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)
