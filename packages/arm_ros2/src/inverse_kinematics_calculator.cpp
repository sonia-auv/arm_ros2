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

#include <arm_ros2/inverse_kinematics_calculator.hpp>
#include <arm_ros2/utility.hpp>
#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace arm_ros2::inverse_kinematics_calculator
{
#define todo(msg)                                  \
    {                                              \
        std::cout << "todo: " << msg << std::endl; \
        exit(1);                                   \
    }
    struct Context
    {
        std::shared_ptr<Config> config;
        rclcpp::Logger &logger;
        double x;
        double y;
        double z;
    };

    namespace Error
    {
        enum T : std::uint8_t
        {
            None,
            HaveNoJoint
        };

        static const char *toString(T &error);
    };  // namespace Error

    /**
     *
     * @brief Calculate the distance between two points.
     */
    static double calculatePointD(const config::joint::Position &positionA, const config::joint::Position &positionB);

    /**
     *
     * @brief Calculate the distance between two joints.
     */
    static double calculateJointD(const config::Joint &jointA, const config::Joint &jointB);

    /**
     *
     * @brief Calculate the maximum distance than the arm can cover.
     * @example
     *
     * =========++===========++=============++
     *    d1           d2            d3      ^ maximum distance
     */
    static double calculateJointDSum(const Context &ctx, Error::T &error);

    /**
     *
     * @brief We determine if the requested position (x, y, z) to reach, is reachable. If so, we return true,
     * otherwise we return false.
     */
    static bool isReachableWorkspace(const Context &ctx, Error::T &error);

    const char *Error::toString(Error::T &error)
    {
        switch (error)
        {
            case Error::T::HaveNoJoint:
                return "The configuration have no joint defined";
            case Error::T::None:
                return "";
            default:
                unreachable();
        }
    }

    double calculatePointD(const config::joint::Position &positionA, const config::joint::Position &positionB)
    {
        double dABX = pow(positionB.getX() - positionA.getX(), 2);
        double dABY = pow(positionB.getY() - positionA.getY(), 2);
        double dABZ = pow(positionB.getZ() - positionA.getZ(), 2);

        return sqrt(dABX + dABY + dABZ);
    }

    double calculateJointD(const config::Joint &jointA, const config::Joint &jointB)
    {
        return calculatePointD(jointA.getPosition(), jointB.getPosition());
    }

    double calculateGripperD(const config::Joint &, const config::Gripper &) { todo("calculateGripperD"); }

    config::joint::Position &getOriginPosition()
    {
        static std::optional<config::joint::Position> originPosition = {};

        if (!originPosition)
        {
            // NOTE: We assume than the origin is at (0, 0, 0).
            originPosition = {0, 0, 0};
        }

        return originPosition.value();
    }

    config::Joint &getOriginJoint()
    {
        static std::optional<config::Joint> originJoint = {};

        if (!originJoint)
        {
            auto joint = config::Joint("origin");
            const auto &originPosition = getOriginPosition();

            joint.setPosition({originPosition.getX(), originPosition.getY(), originPosition.getZ()});

            originJoint = std::move(joint);
        }

        return originJoint.value();
    }

    double calculateJointDSum(const Context &ctx, Error::T &error)
    {
        const auto &joints = ctx.config->getJoints();

        if (joints.empty())
        {
            error = Error::T::HaveNoJoint;

            return 0;
        }

        const auto &lastJoint = joints.back();
        const auto &originJoint = getOriginJoint();

        return calculateJointD(originJoint, lastJoint);
    }

    // The reachable and unreachable cases of the IK problem.
    // The target is unreachable if the distance between the target and the
    // base d is larger than the total sum of all inter-joint distances d >∑n−1
    // i=1 di or smaller than d < d1 − ∑n−1
    // i=2 di . The reachable bounds
    // are shaded in green, while the unreachable cases are outside the
    // outer circle or inside the inner circle shaded in red
    //
    // https://www.andreasaristidou.com/publications/papers/IK_survey.pdf
    bool isReachableWorkspace(const Context &ctx, Error::T &error)
    {
        auto jointDSum = calculateJointDSum(ctx, error);
        auto &originPosition = getOriginPosition();
        config::joint::Position targetPosition = {ctx.x, ctx.y, ctx.z};
        auto targetD = calculatePointD(originPosition, targetPosition);

        return jointDSum >= targetD;
    }

    Result handleCalculateError(const Context &ctx, std::function<Error::T(std::optional<Result> &)> handler)
    {
        std::optional<Result> resultOpt = {};
        auto error = handler(resultOpt);

        if (error != Error::T::None)
        {
            RCLCPP_INFO(ctx.logger, "Get an error: %s", Error::toString(error));
            resultOpt = {};
        }

        Result defaultResult;

        defaultResult.success = false;

        return resultOpt.value_or(std::move(defaultResult));
    }

    Result calculate(std::shared_ptr<Config> config, rclcpp::Logger &logger, double x, double y, double z)
    {
        Context ctx = {std::move(config), logger, x, y, z};

        return handleCalculateError(ctx, [&](std::optional<Result> &resultOpt) {
            auto error = Error::T::None;

            if (!isReachableWorkspace(ctx, error))
            {
                Result result;

                *resultOpt = std::move(result);

                return error;
            }

            return error;
        });
    }
}  // namespace arm_ros2::inverse_kinematics_calculator
