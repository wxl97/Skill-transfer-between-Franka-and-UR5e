#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

namespace pid_example_controller {

class PidExampleController : public controller_interface::MultiInterfaceController<
                                                                    franka_hw::FrankaModelInterface,
                                                                    hardware_interface::EffortJointInterface,
                                                                    franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private:
    std::array<double, 7> saturateTorqueRate(
            const std::array<double, 7>& tau_d_calculated,
            const std::array<double, 7>& tau_J_d);

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    static constexpr double kDeltaTauMax{1.0};
    ros::Duration elapsed_time_;
    std::array<double, 7> initial_pose_{};
    std::array<double, 7> dpose_desired;

    std::vector<double> k_gains_;
    std::vector<double> d_gains_;
    std::array<double, 7> dq_filtered_;

  };

}