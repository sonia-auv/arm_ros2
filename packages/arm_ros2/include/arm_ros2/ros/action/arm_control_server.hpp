/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, SONIA AUV
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <arm_ros2/ros/action/base.hpp>
#include <arm_ros2/ros/service/inverse_kinematics_calculator.hpp>
#include <arm_ros2_interfaces/action/arm_control.hpp>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace arm_ros2::ros::action
{
#undef BaseArmControl
#define BaseArmControl Base<NodeT>

#undef ServiceInverseKinematicsCalculator
#define ServiceInverseKinematicsCalculator service::InverseKinematicsCalculator<NodeT>

    template <typename NodeT>
    class ArmControlServer final : BaseArmControl
    {
        using InterfaceActionArmControl = arm_ros2_interfaces::action::ArmControl;
        using InterfaceActionArmControlGoal = InterfaceActionArmControl::Goal;

        public:
        ArmControlServer(NodeT* node) : BaseArmControl("ArmControl"), _inverseKinematicsCalculatorService(node)
        {
            _server = rclcpp_action::create_server<InterfaceActionArmControl>(
                node, BaseArmControl::getActionName(),
                std::bind(&ArmControlServer::handleGoal, this, node, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ArmControlServer::handleCancel, this, node, std::placeholders::_1),
                std::bind(&ArmControlServer::handleAccepted, this, node, std::placeholders::_1));
        }
        ~ArmControlServer() = default;

        private:
        using GoalHandle = rclcpp_action::ServerGoalHandle<InterfaceActionArmControl>;

        /**
         *
         * @brief Handle goal of `ArmControl` action.
         */
        rclcpp_action::GoalResponse handleGoal(NodeT* node, const rclcpp_action::GoalUUID&,
                                               std::shared_ptr<const InterfaceActionArmControl::Goal>) const
        {
            return BaseArmControl::handleGoal(node, []() { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; });
        }

        /**
         *
         * @brief Handle cancel of `ArmControl` action.
         */
        rclcpp_action::CancelResponse handleCancel(NodeT* node, const std::shared_ptr<GoalHandle>) const
        {
            return BaseArmControl::handleCancel(node, []() { return rclcpp_action::CancelResponse::ACCEPT; });
        }

        /**
         *
         * @brief Handle accept of `ArmControl` action.
         */
        void handleAccepted(NodeT* node, const std::shared_ptr<GoalHandle>) const
        {
            return BaseArmControl::handleAccepted(node, []() { return; });
        }

        rclcpp_action::Server<arm_ros2_interfaces::action::ArmControl>::SharedPtr _server;
        ServiceInverseKinematicsCalculator _inverseKinematicsCalculatorService;
    };

#undef BaseArmControl
#undef ServiceInverseKinematicsCalculator
}  // namespace arm_ros2::ros::action
