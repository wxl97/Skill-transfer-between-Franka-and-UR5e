// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

namespace franka_example_controllers {

class CartesianVelocityExampleController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;
  ros::Subscriber sub_vel_command_;
 
  // double v_x, v_y, v_z, w_x, w_y, w_z;
  std::array<double, 6> target_velocity_;
  std::array<double, 6> current_velocity_;
  std::array<double, 6> start_velocity_;
  ros::Time smoothing_start_time_;
  bool smoothing_active_ = false;

  const double v_max_ = 0.03;
  const double smoothing_time_ = 0.01;  // 1秒平滑过渡

  std::mutex command_mutex_;

  void cartesianvelocitycallback(const geometry_msgs::Twist::ConstPtr& msg);
};

}  // namespace franka_example_controllers
