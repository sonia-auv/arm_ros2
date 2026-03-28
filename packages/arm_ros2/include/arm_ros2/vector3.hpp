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

namespace arm_ros2
{
    template <typename T>
    class Vector3
    {
        public:
        Vector3(T x = {}, T y = {}, T z = {}) : _x(x), _y(y), _z(z) {}
        ~Vector3() = default;

        /**
         *
         * @brief Get `_x` attribute.
         */
        inline T getX() const noexcept { return _x; }

        /**
         *
         * @brief Get `_y` attribute.
         */
        inline T getY() const noexcept { return _y; }

        /**
         *
         * @brief Get `_z` attribute.
         */
        inline T getZ() const noexcept { return _z; }

        /**
         *
         * @brief Add two vectors. 
         */
        inline Vector3<T> add(Vector3<T> &other) noexcept { return Vector3<T>(_x + other._x, _y + other._y, _z + other._z); }

	/**
	 *
	 * @brief Subtract two vectors.
	 */
	inline Vector3<T> sub(Vector3<T> &other) noexcept { return Vector3<T>(_x - other._x, _y - other._y, _z - other._z); }

        private:
        T _x;
        T _y;
        T _z;
    };
}  // namespace arm_ros2
