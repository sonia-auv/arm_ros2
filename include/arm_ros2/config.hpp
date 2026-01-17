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
#include <string>
#include <unordered_map>

namespace arm_ros2
{
    class Config final
    {
        public:
        Config() = default;
        ~Config() = default;

        struct ParserError
        {
            enum class Kind
            {
                AlreadyParsed,
                BadFile,
                Syntax,
                Other
            };

            struct AlreadyParsed;
            struct BadFile;
            struct Syntax;
            struct Other;

            ParserError(Kind kind) { _kind = kind; }
            ~ParserError() = default;

            explicit operator std::string() const noexcept;

            private:
            Kind _kind;
        };

        struct ParserError::AlreadyParsed final : ParserError
        {
            AlreadyParsed() : ParserError(ParserError::Kind::AlreadyParsed) {}

            operator std::string() const noexcept { return "Already parsed"; }
        };

        struct ParserError::BadFile final : ParserError
        {
            BadFile() : ParserError(ParserError::Kind::BadFile) {}

            operator std::string() const noexcept { return "Bad file: not found or something else wrong"; }
        };

        struct ParserError::Syntax final : ParserError
        {
            Syntax(std::string details) : ParserError(ParserError::Kind::Syntax), _details(details) {}

            operator std::string() const noexcept;

            private:
            std::string _details;
        };

        struct ParserError::Other final : ParserError
        {
            Other(std::string message) : ParserError(ParserError::Kind::Other), _message(message) {}

            operator std::string() const noexcept;

            private:
            std::string _message;
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

        /**
         *
         * @brief Get the gripper of the given instance.
         */
        const Gripper& getGripper() const noexcept { return _gripper; }

        private:
        std::unordered_map<std::shared_ptr<std::string>, Joint> _joints;
        Gripper _gripper;
        bool _is_initialized = false;
    };
}  // namespace arm_ros2
