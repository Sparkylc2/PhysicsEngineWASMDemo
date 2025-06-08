//
// Created by Lukas Campbell on 03/06/2025.
//

#pragma once
#include "headers.h"
struct Rigidbody
{
    static float m_DENSITY;
    Vec2 m_pos = {};
    Vec2 m_prev_pos = {};
    Vec2 m_vel = {};
    float m_angle = 0.0;
    float m_angular_vel = 0.0;

    float m_area;
    float m_mass;
    float m_inv_mass = 0;
    float m_rot_inertia = 0;
    float m_inv_rot_inertia = 0;

    float m_width = 0;
    float m_height = 0;
    float m_radius = 0;

    AABB m_aabb;

    std::vector<Vec2> m_verts;
    std::vector<Vec2> m_transformed_verts;

    std::vector<ConstraintRegistry> m_constraints;

    bool m_transform_update_req = true;
    bool m_aabb_update_req = true;

    bool m_is_static = false;

    ShapeType m_shape_type;

    Rigidbody(float radius, bool is_static)
    {
        m_radius = radius;
        m_shape_type = ShapeType::CIRCLE;
        m_area = M_PI * radius * radius;

        if (!m_is_static)
        {
            m_mass = m_area * m_DENSITY;
            m_inv_mass = 1.0 / m_mass;

            m_rot_inertia = 0.5 * m_mass * radius * radius;
            m_inv_rot_inertia = 1.0 / m_rot_inertia;
        }
    }

    Rigidbody(float width, float height, bool is_static)
    {
        m_width = width;
        m_height = height;
        m_radius = 0;
        m_shape_type = ShapeType::BOX;
        m_area = width * height;

        if (!m_is_static)
        {
            m_mass = m_area * m_DENSITY;
            m_inv_mass = 1.0 / m_mass;

            m_rot_inertia = 1.0 / 12.0 * m_mass * (width * width + height * height);
            m_inv_rot_inertia = 1.0 / m_rot_inertia;
        }

        create_box_vertices();
    }

    void transform_verts()
    {
        if (m_shape_type == ShapeType::CIRCLE)
            return;
        if (!m_transform_update_req)
            return;

        PhysicsMath::transform_verts(m_transformed_verts, m_verts, m_pos, m_angle);

        m_aabb_update_req = true;
        m_transform_update_req = false;
    }

    void create_box_vertices()
    {
        float left = -m_width / 2.0;
        float right = m_width / 2.0;
        float top = m_height / 2.0;
        float bottom = -m_height / 2.0;

        m_verts[0] = {left, top};
        m_verts[1] = {right, top};
        m_verts[2] = {right, bottom};
        m_verts[3] = {left, bottom};
    }

    void update(float sf_dt)
    {
        m_aabb_update_req = true;
        m_transform_update_req = true;
        integrate_rk4_pos(sf_dt);
        integrate_angular_euler(sf_dt);
    }

    void integrate_rk4_pos(float &sf_dt)
    {

        Vec2 k1_v = calculate_accel(m_pos) * sf_dt;
        Vec2 k1_r = m_vel * sf_dt;

        Vec2 k2_v = calculate_accel(m_pos + k1_r * 0.5) * sf_dt;
        Vec2 k2_r = (m_vel + k1_v * 0.5) * sf_dt;

        Vec2 k3_v = calculate_accel(m_pos + k2_r * 0.5) * sf_dt;
        Vec2 k3_r = (m_vel + k2_v * 0.5) * sf_dt;

        Vec2 k4_v = calculate_accel(m_pos + k3_r) * sf_dt;
        Vec2 k4_r = (m_vel + k3_v) * sf_dt;

        Vec2 t_k2_r = k2_r * 2.0;
        Vec2 t_k3_r = k3_r * 2.0;

        Vec2 t_k2_v = k2_v * 2.0;
        Vec2 t_k3_v = k3_v * 2.0;

        m_pos += (k1_r + t_k2_r + t_k3_r + k4_r) / 6.0;
        m_vel += (k1_v + t_k2_v + t_k3_v + k4_v) / 6.0;

        m_transform_update_req = true;
    }

    void integrate_angular_euler(float &sf_dt)
    {

        m_angular_vel += calculate_angular_accel() * sf_dt;
        m_angle += m_angular_vel * sf_dt;

        m_transform_update_req = true;
    }

    Vec2 calculate_accel(Vec2 pos)
    {
        Vec2 net_f = {};
        for (auto &constraint : m_constraints)
        {
            Vec2 curr_f = constraint.get_force(this, pos);
            net_f += curr_f;
        }

        return net_f;
    }

    float calculate_angular_accel()
    {

        float net_t = 0.0;
        for (auto &constraint : m_constraints)
        {
            Vec2 lever = constraint.get_application_point(this, m_pos) - m_pos;
            net_t += lever.cross(constraint.get_force(this, m_pos));
        }
        return net_t;
    }
};
