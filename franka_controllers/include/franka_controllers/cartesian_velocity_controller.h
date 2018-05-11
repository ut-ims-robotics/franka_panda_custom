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
#include <geometry_msgs/TwistStamped.h>

namespace franka_controllers {

    pthread_mutex_t cmd_deltas_mutex = PTHREAD_MUTEX_INITIALIZER;

    class CartesianVelocityController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaVelocityCartesianInterface,
            franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

        void update(const ros::Time &, const ros::Duration &period) override;

        void starting(const ros::Time &) override;

        void stopping(const ros::Time &) override;

        // Shared variables
        geometry_msgs::TwistStamped cmd_deltas;


    private:
        franka_hw::FrankaVelocityCartesianInterface *velocity_cartesian_interface_;
        std::unique_ptr <franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
        ros::Duration elapsed_time_;
        ros::NodeHandle n_;
        ros::Subscriber cmd_sub_;
        std::vector<double> current_velocities_;
        std::vector<double> current_accelerations_;
        double acceleration_;
        double acceleration_acc_;


        // ROS subscriber callbacks
        void delta_cmd_cb(const geometry_msgs::TwistStampedConstPtr &msg);
    };

}  // namespace franka_controllers
