
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

    std::vector<ForceRegistry*> m_f_registry;

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

    void transform_verts();
    void create_box_vertices();

    void update(float sf_dt);

    void integrate_pos(float sf_dt);
    void integrate_angular(float sf_dt);

    Vec2 calculate_accel(const Vec2 &pos);

    float calculate_angular_accel();

    void move(const Vec2 &amount);
    void update_aabb();
};
