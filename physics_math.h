//
// Created by Lukas Campbell on 03/06/2025.
//

#pragma once
#include "headers.h"

namespace PhysicsMath
{
    inline float clamp(float val, float min, float max)
    {
        return (val < min) ? min : (val > max) ? max
                                               : val;
    }

    inline bool equals(float a, float b, float epsilon = 0.0001f)
    {
        return std::abs(a - b) < epsilon;
    }

    inline bool equals(const Vec2 &a, const Vec2 &b, float epsilon = 0.0001f)
    {
        return equals(a.m_x, b.m_x, epsilon) && equals(a.m_y, b.m_y, epsilon);
    }

    inline std::vector<Vec2> order_verts_clockwise(std::vector<Vec2> &verts)
    {
        const int len = verts.size();

        Vec2 centroid = Vec2();
        for (const Vec2 &v : verts)
        {
            centroid += v;
        }
        centroid /= len;

        // calculate the angles of each vertex from centroid
        // so we can later sort
        std::vector<float> angles(len);

        for (size_t i = 0; i < len; ++i)
        {
            angles[i] = (verts[i] - centroid).heading();
        }

        // then we sort vertices based on angle
        for (size_t i = 0; i < len; ++i)
        {
            for (size_t j = i + 1; j < len; ++j)
            {
                if (angles[i] > angles[j])
                {
                    Vec2 temp = verts[i];
                    verts[i] = verts[j];
                    verts[j] = temp;

                    float temp_angle = angles[i];
                    angles[i] = angles[j];
                    angles[j] = temp_angle;
                }
            }
        }

        return verts;
    }

    inline void transform_verts(std::vector<Vec2> &dst, const std::vector<Vec2> &src, const Vec2 &t_pos, const float &angle)
    {
        float cos_theta = std::cos(angle);
        float sin_theta = std::sin(angle);

        v128_t cos_theta_v = wasm_f32x4_splat(cos_theta);
        v128_t sin_theta_v = wasm_f32x4_splat(sin_theta);

        v128_t v0 = wasm_v128_load(&src[0]);
        v128_t v1 = wasm_v128_load(&src[2]);

        v128_t xxyy0 = wasm_i32x4_shuffle(v0, v1, 0, 2, 4, 6);
        v128_t xxyy1 = wasm_i32x4_shuffle(v0, v1, 1, 3, 5, 7);

        v128_t x_cos = wasm_f32x4_mul(xxyy0, cos_theta_v);
        v128_t y_sin = wasm_f32x4_mul(xxyy1, sin_theta_v);
        v128_t new_x = wasm_f32x4_sub(x_cos, y_sin);

        v128_t x_sin = wasm_f32x4_mul(xxyy0, sin_theta_v);
        v128_t y_cos = wasm_f32x4_mul(xxyy1, cos_theta_v);
        v128_t new_y = wasm_f32x4_add(x_sin, y_cos);

        v128_t tx_v = wasm_f32x4_splat(t_pos.m_x);
        v128_t ty_v = wasm_f32x4_splat(t_pos.m_y);

        new_x = wasm_f32x4_add(new_x, tx_v);
        new_y = wasm_f32x4_add(new_y, ty_v);

        v128_t xy0 = wasm_i32x4_shuffle(new_x, new_y, 0, 4, 1, 5);
        v128_t xy1 = wasm_i32x4_shuffle(new_x, new_y, 2, 6, 3, 7);

        wasm_v128_store(&dst[0], xy0);
        wasm_v128_store(&dst[2], xy1);
    }
}
