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

#include <arm_ros2/config/joint/angle.hpp>
#include <arm_ros2/config/joint/max_angle.hpp>
#include <arm_ros2/config/joint/position.hpp>
#include <memory>
#include <string>

namespace arm_ros2
{
    class Joint final
    {
        public:
        Joint(std::string name) : _name(std::make_shared<std::string>(name)) {}
        ~Joint() = default;

        /**
         *
         * @brief Get the name of the instance.
         */
        const std::string& getName() const { return *_name; }

        /**
         *
         * @brief Get the position of the instance.
         */
        const Position& getPosition() const { return _position; }

        /**
         *
         * @brief Get the angle of the instance.
         */
        const Angle& getAngle() const { return _angle; }

        /**
         *
         * @brief Get the max angle of the instance.
         */
        const MaxAngle& getMaxAngle() const { return _maxAngle; }

        /**
         *
         * @brief Set the position attribute with the given x, y and coordinate.
         * @param x Initial absolute position of X axis
         * @param y Initial absolute position of Y axis
         * @param z Initial absolute position of Z axis
         */
        void setPosition(float x = 0, float y = 0, float z = 0) { _position = Position(x, y, z); }

        /**
         *
         * @brief Set the angle attribute with the given x, y and z coordinate.
         * @param x Initial tilt angle in degree of X axis
         * @param y Initial tilt angle in degree of Y axis
         * @param z Initial tilt angle in degree of Z axis
         */
        void setAngle(float x = 0, float y = 0, float z = 0) { _angle = Angle(x, y, z); }

        /**
         *
         * @brief Set the max angle attribute with the given x, y and coordinate.
         * @param x Maximum tilt angle in degree of X axis
         * @param y Maximum tilt angle in degree of Y axis
         * @param z Maximum tilt angle in degree of Z axis
         */
        void setMaxAngle(float x = 0, float y = 0, float z = 0) { _maxAngle = MaxAngle(x, y, z); }

        private:
        std::shared_ptr<std::string> _name;
        Position _position;
        Angle _angle;
        MaxAngle _maxAngle;
    };
}  // namespace arm_ros2
