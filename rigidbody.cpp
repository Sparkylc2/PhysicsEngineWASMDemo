//
// Created by Lukas Campbell on 03/06/2025.
//

#include "headers.h"

void Rigidbody::transform_verts()
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
        for (int i = 0; i < m_verts.size(); ++i)
        {
        }
    }

    m_aabb_update_req = true;
    m_transform_update_req = false;
}

void Rigidbody::create_box_vertices()
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

void Rigidbody::update(float sf_dt)
{
    if (DEBUG)
        std::cout << "body.update(): pos x y " << m_pos.m_x << ", " << m_pos.m_y << std::endl;
    m_aabb_update_req = true;
    m_transform_update_req = true;
    integrate_pos(sf_dt);
    integrate_angular(sf_dt);
    update_aabb();
}

void Rigidbody::integrate_pos(float sf_dt)
{
    Vec2 accel = calculate_accel(m_pos);
    m_vel += accel * sf_dt;
    m_pos += m_vel * sf_dt;

    if (DEBUG)
    {
        std::cout << "New position: " << m_pos.m_x << ", " << m_pos.m_y << std::endl;
    }

    m_transform_update_req = true;
}

void Rigidbody::integrate_angular(float sf_dt)
{
    float angular_accel = calculate_angular_accel();
    m_angular_vel += angular_accel * sf_dt;
    m_angle += m_angular_vel * sf_dt;

    m_transform_update_req = true;
}
Vec2 Rigidbody::calculate_accel(const Vec2 &pos)
{
    Vec2 net_f = Vec2();

    if (m_is_static)
    {
        return net_f;
    }

    for (const auto& f : m_f_registry)
    {
        net_f += f->get_force(this, pos);
    }

    return net_f * m_inv_mass;
}

float Rigidbody::calculate_angular_accel()
{
    float net_t = 0.0f;

    if (m_is_static)
    {
        return net_t;
    }

    for (const auto& f : m_f_registry)
    {
        Vec2 curr_f = f->get_force(this, m_pos);
        Vec2 lever_arm = f->get_application_point(this, m_pos) - m_pos;
        net_t += lever_arm.cross(curr_f);
    }

    return net_t * m_inv_rot_inertia;
}

void Rigidbody::move(const Vec2 &amount)
{
    m_pos += amount;
    m_transform_update_req = true;
    m_aabb_update_req = true;
}

void Rigidbody::update_aabb()
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


void Rigidbody::update_geometry(float width, float height)
{
    m_width = width;
    m_height = height;
    m_radius = 0.0f;
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
    update_aabb();
}
void Rigidbody::update_geometry(float radius)
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

    update_aabb();
}


bool Rigidbody::contains(const Vec2 &point) {
    if (this->m_shape_type == ShapeType::CIRCLE) {
        return this->contains_circle(point);
    } else {
        return this->contains_polygon(point);
    }
}

bool Rigidbody::contains_polygon(const Vec2& point) {
    bool inside = false;

    for (size_t i = 0, j = m_transformed_verts.size() - 1; i < m_transformed_verts.size(); j = i++) {
        const Vec2& v1 = m_transformed_verts[i];
        const Vec2& v2 = m_transformed_verts[j];

        if ((v1.m_y > point.m_y) != (v2.m_y > point.m_y) &&
            (point.m_x < (v2.m_x - v1.m_x) * (point.m_y - v1.m_y) / (v2.m_y - v1.m_y) + v1.m_x)) {
            inside = !inside;
        }
    }
    return inside;
}


bool Rigidbody::contains_circle(const Vec2& point) {
    float distance = (this->m_pos - point).len();
    return (distance <= this->m_radius);
}



