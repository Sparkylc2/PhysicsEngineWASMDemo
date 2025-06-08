//
// Created by Lukas Campbell on 03/06/2025.
//

#pragma once
#include "headers.h"
struct Vec2
{
    float m_x = 0.0f;
    float m_y = 0.0f;

    Vec2() = default;
    Vec2(float x_, float y_) : m_x(x_), m_y(y_) {}

    inline Vec2 operator+(const Vec2 &other) const
    {
        return Vec2(m_x + other.m_x, m_y + other.m_y);
    }

    inline Vec2 &operator+=(const Vec2 &other)
    {
        m_x += other.m_x;
        m_y += other.m_y;
        return *this;
    }

    inline Vec2 operator-(const Vec2 &other) const
    {
        return Vec2(m_x - other.m_x, m_y - other.m_y);
    }

    inline Vec2 &operator-=(const Vec2 &other)
    {
        m_x -= other.m_x;
        m_y -= other.m_y;
        return *this;
    }

    inline Vec2 operator*(float scalar) const
    {
        return Vec2(m_x * scalar, m_y * scalar);
    }

    inline Vec2 &operator*=(float scalar)
    {
        m_x *= scalar;
        m_y *= scalar;
        return *this;
    }

    inline Vec2 operator/(float scalar) const
    {
        return Vec2(m_x / scalar, m_y / scalar);
    }

    inline Vec2 &operator/=(float scalar)
    {
        m_x /= scalar;
        m_y /= scalar;
        return *this;
    }

    inline bool operator==(const Vec2 &other) const
    {
        return m_x == other.m_x && m_y == other.m_y;
    }

    inline bool operator!=(const Vec2 &other) const
    {
        return !(*this == other);
    }

    inline float dot(const Vec2 &other) const
    {
        return m_x * other.m_x + m_y * other.m_y;
    }

    inline Vec2 hadamard(const Vec2 &other) const
    {
        return Vec2(m_x * other.m_x, m_y * other.m_y);
    }

    inline float inv_len() const
    {
        return 1.0f / len();
    }

    inline float len() const
    {

        v128_t vec = wasm_f32x4_make(m_x, m_y, 0.0f, 0.0f);
        v128_t mul = wasm_f32x4_mul(vec, vec);
        float len_sq = wasm_f32x4_extract_lane(mul, 0) + wasm_f32x4_extract_lane(mul, 1);
        v128_t len_sq_vec = wasm_f32x4_splat(len_sq);
        v128_t sqrt = wasm_f32x4_sqrt(len_sq_vec);
        return wasm_f32x4_extract_lane(sqrt, 0);
    }

    inline float heading() const
    {
        return atan2f(m_y, m_x);
    }

    inline void normalize()
    {
        float inv_length = inv_len();
        m_x *= inv_length;
        m_y *= inv_length;
    }

    inline float cross(Vec2 &v)
    {
        return m_x * v.m_y - m_y * v.m_x;
    }

    inline void zero()
    {
        m_x = 0.0;
        m_y = 0.0;
    }
};
