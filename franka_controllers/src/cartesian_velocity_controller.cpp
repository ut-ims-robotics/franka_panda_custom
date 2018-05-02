// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_controllers/cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_controllers {

    bool CartesianVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                           ros::NodeHandle &node_handle) {
        n_ = node_handle;

        std::string velocity_in_topic;
        if (!node_handle.getParam("/panda_cartesian_velocity_controller/velocity_in_topic", velocity_in_topic)) {
            ROS_ERROR_STREAM("[CartesianVelocityController] YAML config file does not contain parameter 'velocity_in_topic'");
            return false;
        }
        cmd_sub_ = node_handle.subscribe(velocity_in_topic, 1,
                                         &CartesianVelocityController::delta_cmd_cb, this);

        if (!node_handle.getParam("/panda_cartesian_velocity_controller/acceleration", acceleration_)) {
            ROS_ERROR_STREAM("[CartesianVelocityController] YAML config file does not contain parameter 'acceleration'");
            return false;
        }

        //Initial values
        cmd_deltas.twist.linear.x = 0.0;
        cmd_deltas.twist.linear.y = 0.0;
        cmd_deltas.twist.linear.z = 0.0;
        cmd_deltas.twist.angular.x = 0.0;
        cmd_deltas.twist.angular.y = 0.0;
        cmd_deltas.twist.angular.z = 0.0;

        for (int i = 0; i < 6; i++) {
            current_velocities_.push_back(0.0);
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
            return false;
        }

        velocity_cartesian_interface_ =
                robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
        if (velocity_cartesian_interface_ == nullptr) {
            ROS_ERROR(
                    "CartesianVelocityController: Could not get Cartesian velocity interface from "
                            "hardware");
            return false;
        }
        try {
            velocity_cartesian_handle_.reset(new franka_hw::FrankaCartesianVelocityHandle(
                    velocity_cartesian_interface_->getHandle(arm_id + "_robot")));
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM(
                    "CartesianVelocityController: Exception getting Cartesian handle: " << e.what());
            return false;
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("CartesianVelocityController: Could not get state interface from hardware");
            return false;
        }

        return true;
    }

    void CartesianVelocityController::starting(const ros::Time & /* time */) {
        elapsed_time_ = ros::Duration(0.0);
    }

    void CartesianVelocityController::update(const ros::Time & /* time */,
                                             const ros::Duration &period) {
        pthread_mutex_lock(&cmd_deltas_mutex);
        geometry_msgs::TwistStamped cmd_deltas_ = cmd_deltas;
        pthread_mutex_unlock(&cmd_deltas_mutex);

        //Pull velocities to vector
        std::vector<double> input_velocities;
        input_velocities.push_back(cmd_deltas_.twist.linear.x);
        input_velocities.push_back(cmd_deltas_.twist.linear.y);
        input_velocities.push_back(cmd_deltas_.twist.linear.z);
        input_velocities.push_back(cmd_deltas_.twist.angular.x);
        input_velocities.push_back(cmd_deltas_.twist.angular.y);
        input_velocities.push_back(cmd_deltas_.twist.angular.z);

        //Iterate over every dimension and update current speed values
        for (int i = 0; i < input_velocities.size(); i++) {
            if (std::abs(current_velocities_.at(i) - input_velocities.at(i)) > acceleration_) {
                if (current_velocities_.at(i) > input_velocities.at(i)) {
                    current_velocities_.at(i) = current_velocities_.at(i) - acceleration_;
                } else if (current_velocities_.at(i) < input_velocities.at(i)) {
                    current_velocities_.at(i) = current_velocities_.at(i) + acceleration_;
                } else {
                    current_velocities_.at(i) = input_velocities.at(i);
                }
            } else {
                current_velocities_.at(i) = input_velocities.at(i);
            }
        }


        std::array<double, 6> command = {{current_velocities_.at(0), current_velocities_.at(1), current_velocities_.at(
                2), current_velocities_.at(3), current_velocities_.at(4), current_velocities_.at(5)}};


        velocity_cartesian_handle_->setCommand(command);
    }

    void CartesianVelocityController::stopping(const ros::Time & /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    }

    void CartesianVelocityController::delta_cmd_cb(const geometry_msgs::TwistStampedConstPtr &msg) {
        pthread_mutex_lock(&cmd_deltas_mutex);
        cmd_deltas = *msg;
        pthread_mutex_unlock(&cmd_deltas_mutex);
    }

}  // namespace franka_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::CartesianVelocityController,
        controller_interface::ControllerBase
)
