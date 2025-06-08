//
// Created by Lukas Campbell on 03/06/2025.
//

#pragma once
#include "headers.h"

struct Rigidbody
{
    static constexpr float m_DENSITY = 1.0f;

    Vec2 m_pos = {};
    Vec2 m_prev_pos = {};
    Vec2 m_vel = {};
    float m_angle = 0.0f;
    float m_angular_vel = 0.0f;

    float m_area = 0.0f;
    float m_mass = 0.0f;
    float m_inv_mass = 0.0f;
    float m_rot_inertia = 0.0f;
    float m_inv_rot_inertia = 0.0f;

    float m_width = 0.0f;
    float m_height = 0.0f;
    float m_radius = 0.0f;

    AABB m_aabb;

    std::vector<Vec2> m_verts;
    std::vector<Vec2> m_transformed_verts;

    bool m_transform_update_req = true;
    bool m_aabb_update_req = true;

    bool m_is_static = false;

    ShapeType m_shape_type;

    float m_restitution = 0.5f;
    float m_static_friction = 0.8f;
    float m_kinetic_friction = 0.3f;

    Rigidbody(float radius, bool is_static) : m_is_static(is_static)
    {
        m_radius = radius;
        m_shape_type = ShapeType::CIRCLE;
        m_area = M_PI * radius * radius;

        if (!m_is_static)
        {
            m_mass = m_area * m_DENSITY;
            m_inv_mass = 1.0f / m_mass;

            m_rot_inertia = 0.5f * m_mass * radius * radius;
            m_inv_rot_inertia = 1.0f / m_rot_inertia;
        }
    }

    Rigidbody(float width, float height, bool is_static) : m_is_static(is_static)
    {
        m_width = width;
        m_height = height;
        m_radius = 0;
        m_shape_type = ShapeType::BOX;
        m_area = width * height;

        if (!m_is_static)
        {
            m_mass = m_area * m_DENSITY;
            m_inv_mass = 1.0f / m_mass;

            m_rot_inertia = (1.0f / 12.0f) * m_mass * (width * width + height * height);
            m_inv_rot_inertia = 1.0f / m_rot_inertia;
        }

        create_box_vertices();
    }

    void transform_verts()
    {
        if (m_shape_type == ShapeType::CIRCLE)
            return;
        if (!m_transform_update_req)
            return;

        if (m_transformed_verts.size() != m_verts.size())
            m_transformed_verts.resize(m_verts.size());

        if (m_verts.size() == 4)
        {
            PhysicsMath::transform_verts(m_transformed_verts, m_verts, m_pos, m_angle);
        }
        else
        {
            float cos_theta = std::cos(m_angle);
            float sin_theta = std::sin(m_angle);

            for (size_t i = 0; i < m_verts.size(); ++i)
            {
                float x = m_verts[i].m_x * cos_theta - m_verts[i].m_y * sin_theta;
                float y = m_verts[i].m_x * sin_theta + m_verts[i].m_y * cos_theta;

                m_transformed_verts[i].m_x = x + m_pos.m_x;
                m_transformed_verts[i].m_y = y + m_pos.m_y;
            }
        }

        m_aabb_update_req = true;
        m_transform_update_req = false;
    }

    void create_box_vertices()
    {
        float left = -m_width / 2.0f;
        float right = m_width / 2.0f;
        float top = m_height / 2.0f;
        float bottom = -m_height / 2.0f;

        m_verts.resize(4);
        m_verts[0] = Vec2(left, top);
        m_verts[1] = Vec2(right, top);
        m_verts[2] = Vec2(right, bottom);
        m_verts[3] = Vec2(left, bottom);
    }

    void update(float sf_dt)
    {
        m_aabb_update_req = true;
        m_transform_update_req = true;
        integrate_pos(sf_dt);
        integrate_angular(sf_dt);
        update_aabb();
    }

    void integrate_pos(float sf_dt)
    {
        Vec2 accel = calculate_accel(m_pos);
        m_vel += accel * sf_dt;
        m_pos += m_vel * sf_dt;

        m_transform_update_req = true;
    }

    void integrate_angular(float sf_dt)
    {
        float angular_accel = calculate_angular_accel();
        m_angular_vel += angular_accel * sf_dt;
        m_angle += m_angular_vel * sf_dt;

        m_transform_update_req = true;
    }

    Vec2 calculate_accel(const Vec2 &pos)
    {
        if (m_is_static)
            return Vec2(0, 0);
        return Vec2(0, 9.81f);
    }

    float calculate_angular_accel()
    {
        return 0.0f;
    }

    void move(const Vec2 &amount)
    {
        m_pos += amount;
        m_transform_update_req = true;
        m_aabb_update_req = true;
    }

    void update_aabb()
    {
        if (!m_aabb_update_req)
            return;

        if (m_shape_type == ShapeType::CIRCLE)
        {
            m_aabb.m_min = Vec2(m_pos.m_x - m_radius, m_pos.m_y - m_radius);
            m_aabb.m_max = Vec2(m_pos.m_x + m_radius, m_pos.m_y + m_radius);
        }
        else
        {
            transform_verts();

            float min_x = FLT_MAX, min_y = FLT_MAX;
            float max_x = -FLT_MAX, max_y = -FLT_MAX;

            for (const Vec2 &v : m_transformed_verts)
            {
                if (v.m_x < min_x)
                    min_x = v.m_x;
                if (v.m_y < min_y)
                    min_y = v.m_y;
                if (v.m_x > max_x)
                    max_x = v.m_x;
                if (v.m_y > max_y)
                    max_y = v.m_y;
            }

            m_aabb.m_min = Vec2(min_x, min_y);
            m_aabb.m_max = Vec2(max_x, max_y);
        }

        m_aabb_update_req = false;
    }
};
