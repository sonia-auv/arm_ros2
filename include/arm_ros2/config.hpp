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

#include <yaml-cpp/yaml.h>

#include <arm_ros2/config/gripper.hpp>
#include <arm_ros2/config/joint.hpp>
#include <memory>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <variant>

namespace arm_ros2
{
    class Config final
    {
        public:
        Config() = default;
        ~Config() = default;

        struct ParserError final
        {
            struct _AlreadyParsed final
            {
                _AlreadyParsed() {}

                operator std::string() const noexcept { return "Already parsed"; }
            };

            struct _BadFile final
            {
                _BadFile() {}

                operator std::string() const noexcept { return "Bad file: not found or something else wrong"; }
            };

            struct _Syntax final
            {
                _Syntax(std::string details) : _details(details) {}

                operator std::string() const noexcept;

                private:
                std::string _details;
            };

            struct _BadConfig final
            {
                _BadConfig(std::string details) : _details(details) {}

                operator std::string() const noexcept;

                private:
                std::string _details;
            };

            struct _Other final
            {
                _Other(std::string message) : _message(message) {}

                operator std::string() const noexcept;

                private:
                std::string _message;
            };

            using Value = std::variant<_AlreadyParsed, _BadFile, _Syntax, _BadConfig, _Other>;

            ParserError(Value value) : _value(value) {}
            ~ParserError() = default;

            static ParserError AlreadyParsed() { return ParserError(Value(std::in_place_type<_AlreadyParsed>)); }

            static ParserError BadFile() { return ParserError(Value(std::in_place_type<_BadFile>)); }

            static ParserError Syntax(_Syntax value) { return ParserError(Value(std::in_place_type<_Syntax>, value)); }

            static ParserError BadConfig(_BadConfig value)
            {
                return ParserError(Value(std::in_place_type<_BadConfig>, value));
            }

            static ParserError Other(_Other value) { return ParserError(Value(std::in_place_type<_Other>, value)); }

            explicit operator std::string() const noexcept;

            private:
            Value _value;
        };

        using ParserErrorOr = std::optional<ParserError>;

        /**
         *
         * @brief Parse the configuration.
         */
        [[nodiscard]] ParserErrorOr parse(const std::string& filename) noexcept;

        /**
         *
         * @brief Get the joints of the given instance.
         */
        const std::unordered_map<std::shared_ptr<std::string>, Joint>& getJoints() const noexcept { return _joints; }

        void insertJoint(Joint joint) noexcept { _joints.insert({joint.getNamePtr(), joint}); }

        /**
         *
         * @brief Get the gripper of the given instance.
         */
        const Gripper& getGripper() const noexcept { return _gripper; }

        private:
        template <class CoordinateCompatible>
        [[nodiscard]] ParserErrorOr parseJointCoordinate(const YAML::Node& node, CoordinateCompatible& coordinate,
                                                         const char* key) noexcept;
        [[nodiscard]] ParserErrorOr parseJointMaxAngle(const YAML::Node& node, MaxAngle& maxAngle) noexcept;
        [[nodiscard]] ParserErrorOr parseJointAngle(const YAML::Node& node, Angle& angle) noexcept;
        [[nodiscard]] ParserErrorOr parseJointPosition(const YAML::Node& node, Position& position) noexcept;
        [[nodiscard]] ParserErrorOr parseJoint(const YAML::Node& node) noexcept;
        [[nodiscard]] ParserErrorOr parseJoints(const YAML::Node& node) noexcept;
        [[nodiscard]] ParserErrorOr parseGripper(const YAML::Node& node) noexcept;

        std::unordered_map<std::shared_ptr<std::string>, Joint> _joints;
        Gripper _gripper;
        bool _is_initialized = false;
    };
}  // namespace arm_ros2
