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

#include <arm_ros2/config.hpp>
#include <arm_ros2/utility.hpp>
#include <iostream>
#include <sstream>

namespace arm_ros2
{
    Config::ParserError::operator std::string() const noexcept
    {
        return std::visit(overloaded{
                              [](Config::ParserError::_AlreadyParsed value) -> std::string {
                                  return static_cast<const Config::ParserError::_AlreadyParsed &>(value);
                              },
                              [](Config::ParserError::_BadFile value) -> std::string {
                                  return static_cast<const Config::ParserError::_BadFile &>(value);
                              },
                              [](Config::ParserError::_Syntax value) -> std::string {
                                  return static_cast<const Config::ParserError::_Syntax &>(value);
                              },
                              [](Config::ParserError::_BadConfig value) -> std::string {
                                  return static_cast<const Config::ParserError::_BadConfig &>(value);
                              },
                              [](Config::ParserError::_Other value) -> std::string {
                                  return static_cast<const Config::ParserError::_Other &>(value);
                              },
                          },
                          _value);
    }

    Config::ParserError::_Syntax::operator std::string() const noexcept
    {
        std::stringstream ss;

        ss << "Syntax: " << _details;

        return ss.str();
    }

    Config::ParserError::_BadConfig::operator std::string() const noexcept
    {
        std::stringstream ss;

        ss << "BadConfig: " << _details;

        return ss.str();
    }

    Config::ParserError::_Other::operator std::string() const noexcept
    {
        std::stringstream ss;

        ss << "Other: " << _message;

        return ss.str();
    }

    [[nodiscard]] Config::ParserErrorOr Config::parseJoint(const YAML::Node &node) noexcept
    {
        std::cout << node << "Hello" << std::endl;
        return {};
    }

    [[nodiscard]] Config::ParserErrorOr Config::parseJoints(const YAML::Node &node) noexcept
    {
        const YAML::Node joints = node["joints"];

        if (!joints)
        {
            return Config::ParserError::BadConfig(
                Config::ParserError::_BadConfig("Expected `joints` key in the configuration"));
        }
        else if (!joints.IsMap())
        {
            return Config::ParserError::BadConfig(Config::ParserError::_BadConfig("Expected a map for `joints` key"));
        }

        for (const std::pair<YAML::Node, YAML::Node> &keyValue : node)
        {
            auto parseError = parseJoint(keyValue.second);

            if (parseError != std::nullopt)
            {
                return parseError;
            }
        }

        return {};
    }

    [[nodiscard]] Config::ParserErrorOr Config::parseGripper(const YAML::Node &node) noexcept
    {
        std::cout << node << "Hello" << std::endl;
        return {};
    }

    Config::ParserErrorOr Config::parse(const std::string &filename) noexcept
    {
        if (_is_initialized)
        {
            return Config::ParserError::AlreadyParsed();
        }

        YAML::Node node;

        try
        {
            node = YAML::LoadFile(filename);
        }
        catch (const YAML::BadFile &)
        {
            return Config::ParserError::BadFile();
        }
        catch (const YAML::ParserException &e)
        {
            return Config::ParserError::Syntax(Config::ParserError::_Syntax(e.what()));
        }
        catch (const std::runtime_error &e)
        {
            return Config::ParserError::Other(Config::ParserError::_Other(e.what()));
        }
        catch (...)
        {
            return Config::ParserError::Other(Config::ParserError::_Other("Unknown error"));
        }

        std::function<Config::ParserErrorOr(Config &, YAML::Node &)> parseFunctions[] = {
            &Config::parseJoints,
            &Config::parseGripper,
        };

        for (const auto &parseFunction : parseFunctions)
        {
            auto parserError = parseFunction(*this, node);

            if (parserError != std::nullopt)
            {
                return parserError;
            }
        }

        return {};
    }
}  // namespace arm_ros2
