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

namespace arm_ros2
{
    template <typename V>
    class Coordinate
    {
        public:
        Coordinate(V x = {}, V y = {}, V z = {}) : _x(x), _y(y), _z(z) {}
        Coordinate(const Coordinate&) = default;
        ~Coordinate() = default;

        /**
         *
         * @brief Get _x attribute.
         */
        inline V getX() const noexcept { return _x; }

        /**
         *
         * @brief Get _y attribute.
         */
        inline V getY() const noexcept { return _y; }

        /**
         *
         * @brief Get _z attribute.
         */
        inline V getZ() const noexcept { return _z; }

        /**
         *
         * @brief Set _x attribute.
         */
        inline void setX(V x) noexcept { _x = x; }

        /**
         *
         * @brief Set _y attribute.
         */
        inline void setY(V y) noexcept { _y = y; }

        /**
         *
         * @brief Set _z attribute.
         */
        inline void setZ(V z) noexcept { _z = z; }

        private:
        V _x;
        V _y;
        V _z;
    };
}  // namespace arm_ros2
