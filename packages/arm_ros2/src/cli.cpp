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

#include <yaml-cpp/yaml.h>

#include <arm_ros2/cli.hpp>
#include <arm_ros2/config.hpp>
#include <arm_ros2/ros/node.hpp>
#include <iostream>

namespace arm_ros2::cli
{
    static void emitError(std::string_view message);

    int run(int argc, char **argv)
    {
#define DEFAULT_CONFIG_PATH "config/config.yaml"

        const char *configPath;

        if (argc < 2)
        {
            configPath = DEFAULT_CONFIG_PATH;
        }
        else
        {
            configPath = argv[1];
        }

        auto config = Config();
        auto parserError = config.parse(configPath);

        if (parserError.has_value())
        {
            auto parserErrorMessage = static_cast<std::string>(parserError.value());

            emitError(parserErrorMessage);

            return EXIT_FAILURE;
        }

        rclcpp::init(argc, argv);

        auto res = ros::node::init(config);

        rclcpp::shutdown();

        return res;

#undef DEFAULT_CONFIG_PATH
    }

    void emitError(std::string_view message) { std::cerr << "\x1b[31mError\x1b[0m: " << message << std::endl; }
}  // namespace arm_ros2::cli
