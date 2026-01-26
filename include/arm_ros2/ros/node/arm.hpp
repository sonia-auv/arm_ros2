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

#include <arm_ros2/config.hpp>
#include <arm_ros2/ros/node/joint.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace arm_ros2::ros::node
{
    class Arm final : public rclcpp::Node
    {
        public:
        ~Arm() = default;

        /**
         *
         * @brief Return a static initialization of the Arm class.
         */
        static std::shared_ptr<Arm> getInstance();

        /**
         *
         * @brief Set ROS joint nodes from a given configuration.
         * @param config Configuration of the arm.
         */
        void setJointNodes(const Config& config);

        private:
        Arm() : rclcpp::Node("arm") {}

        std::vector<std::shared_ptr<Joint>> _jointNodes;
    };
}  // namespace arm_ros2::ros::node
