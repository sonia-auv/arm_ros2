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

#include <arm_ros2/config.hpp>
#include <arm_ros2/utility.hpp>

namespace arm_ros2
{
    Config::ParserError::operator std::string() const noexcept
    {
        switch (_kind)
        {
            case Config::ParserError::Kind::AlreadyParsed:
                return static_cast<std::string>(static_cast<const Config::ParserError::AlreadyParsed &>(*this));
            case Config::ParserError::Kind::BadFile:
                return static_cast<std::string>(static_cast<const Config::ParserError::BadFile &>(*this));
            case Config::ParserError::Kind::Syntax:
                return static_cast<std::string>(static_cast<const Config::ParserError::Syntax &>(*this));
            case Config::ParserError::Kind::Other:
                return static_cast<std::string>(static_cast<const Config::ParserError::Other &>(*this));
            default:
                unreachable();
        }
    }

    Config::ParserErrorOr Config::parse(const std::string &filename) noexcept
    {
        if (_is_initialized)
        {
            return Config::ParserError::AlreadyParsed();
        }

        try
        {
            auto node = YAML::LoadFile(filename);
        }
        catch (const YAML::BadFile &)
        {
            return Config::ParserError::BadFile();
        }
        catch (const YAML::ParserException &e)
        {
            return Config::ParserError::Syntax(e.what());
        }
        catch (const std::runtime_error &e)
        {
            return Config::ParserError::Other(e.what());
        }
        catch (...)
        {
            return Config::ParserError::Other("Unknown error");
        }

        // TODO:

        return {};
    }
}  // namespace arm_ros2
