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

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace arm_ros2::ros::action
{
    template <typename NodeT, typename ActionGoal>
    class Base
    {
        public:
        Base(const char *actionName) : _actionName(actionName) {}
        ~Base() = default;

        /**
         *
         * @brief Get defined action name.
         */
        const char *getActionName() const noexcept { return _actionName; }

        protected:
        using GoalHandler = std::function<rclcpp_action::GoalResponse()>;
        using CancelHandler = std::function<rclcpp_action::CancelResponse()>;
        using AcceptedHandler = std::function<void()>;

        /**
         *
         * @brief Handle goal of `ArmControl` action.
         */
        rclcpp_action::GoalResponse handleGoal(NodeT *node, GoalHandler handler) const
        {
            RCLCPP_INFO(node->get_logger(), "Received goal request: %s", _actionName);

            return handler();
        }

        /**
         *
         * @brief Handle cancel of `ArmControl` action.
         */
        rclcpp_action::CancelResponse handleCancel(NodeT *node, CancelHandler handler) const
        {
            RCLCPP_INFO(node->get_logger(), "Received cancel request: %s", _actionName);

            return handler();
        }

        /**
         *
         * @brief Handle accept of `ArmControl` action.
         */
        void handleAccepted(NodeT *node, AcceptedHandler handler) const
        {
            RCLCPP_INFO(node->get_logger(), "Received accepted request: %s", _actionName);

            return handler();
        }

        private:
        const char *_actionName;
    };
}  // namespace arm_ros2::ros::action
