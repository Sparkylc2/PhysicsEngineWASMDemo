//
// Created by Lukas Campbell on 08/06/2025.
//

#pragma once
#include "headers.h"

namespace CollisionResolution
{
    struct CollisionManifold
    {
        Rigidbody *m_rb_a;
        Rigidbody *m_rb_b;
        Vec2 m_normal;
        float m_depth;
        std::vector<Vec2> m_points_of_contact;
        int m_contact_count;

        CollisionManifold(Rigidbody *rb_a, Rigidbody *rb_b, const Collisions::CollisionResult &result)
            : m_rb_a(rb_a), m_rb_b(rb_b), m_normal(result.m_normal), m_depth(result.m_depth),
              m_points_of_contact(result.m_points_of_contact), m_contact_count(result.m_contact_count)
        {
        }
    };

    inline void separate_bodies(Rigidbody &rb_a, Rigidbody &rb_b, const Vec2 &mtv)
    {
        if (rb_a.m_is_static)
        {
            rb_b.move(mtv);
        }
        else if (rb_b.m_is_static)
        {
            rb_a.move(mtv * -1.0f);
        }
        else
        {
            rb_a.move(mtv * -0.5f);
            rb_b.move(mtv * 0.5f);
        }
    }

    inline void resolve_collision_with_friction(CollisionManifold &manifold)
    {
        Rigidbody &rb_a = *manifold.m_rb_a;
        Rigidbody &rb_b = *manifold.m_rb_b;

        float inv_mass_a = rb_a.m_inv_mass;
        float inv_mass_b = rb_b.m_inv_mass;
        float inv_rot_inertia_a = rb_a.m_inv_rot_inertia;
        float inv_rot_inertia_b = rb_b.m_inv_rot_inertia;

        Vec2 pos_a = rb_a.m_pos;
        Vec2 pos_b = rb_b.m_pos;
        Vec2 vel_a = rb_a.m_vel;
        Vec2 vel_b = rb_b.m_vel;
        float ang_vel_a = rb_a.m_angular_vel;
        float ang_vel_b = rb_b.m_angular_vel;

        Vec2 &normal = manifold.m_normal;
        int contact_count = manifold.m_contact_count;

        float restitution = std::min(rb_a.m_restitution, rb_b.m_restitution);
        float static_friction = (rb_a.m_static_friction + rb_b.m_static_friction) * 0.5f;
        float kinetic_friction = (rb_a.m_kinetic_friction + rb_b.m_kinetic_friction) * 0.5f;

        std::vector<Vec2> impulse_list(contact_count);
        std::vector<Vec2> ra_list(contact_count);
        std::vector<Vec2> rb_list(contact_count);
        std::vector<Vec2> friction_impulse_list(contact_count);
        std::vector<float> j_list(contact_count);

        for (int i = 0; i < contact_count; ++i)
        {
            Vec2 ra = manifold.m_points_of_contact[i] - pos_a;
            Vec2 rb = manifold.m_points_of_contact[i] - pos_b;

            ra_list[i] = ra;
            rb_list[i] = rb;

            Vec2 ra_perp(-ra.m_y, ra.m_x);
            Vec2 rb_perp(-rb.m_y, rb.m_x);

            Vec2 ang_linear_vel_a = ra_perp * ang_vel_a;
            Vec2 ang_linear_vel_b = rb_perp * ang_vel_b;

            Vec2 relative_vel = (vel_b + ang_linear_vel_b) - (vel_a + ang_linear_vel_a);
            float contact_vel_mag = relative_vel.dot(normal);

            if (contact_vel_mag > 0.0f)
                continue;

            float ra_perp_dot_n = ra_perp.dot(normal);
            float rb_perp_dot_n = rb_perp.dot(normal);

            float denom = inv_mass_a + inv_mass_b +
                          (ra_perp_dot_n * ra_perp_dot_n) * inv_rot_inertia_a +
                          (rb_perp_dot_n * rb_perp_dot_n) * inv_rot_inertia_b;

            float j = -(1.0f + restitution) * contact_vel_mag;
            j /= denom;
            j /= static_cast<float>(contact_count);

            j_list[i] = j;
            impulse_list[i] = normal * j;
        }

        for (int i = 0; i < contact_count; ++i)
        {
            Vec2 &impulse = impulse_list[i];
            Vec2 &ra = ra_list[i];
            Vec2 &rb = rb_list[i];

            vel_a -= impulse * inv_mass_a;
            vel_b += impulse * inv_mass_b;

            ang_vel_a -= ra.cross(impulse) * inv_rot_inertia_a;
            ang_vel_b += rb.cross(impulse) * inv_rot_inertia_b;
        }

        for (int i = 0; i < contact_count; ++i)
        {
            Vec2 ra = manifold.m_points_of_contact[i] - pos_a;
            Vec2 rb = manifold.m_points_of_contact[i] - pos_b;

            ra_list[i] = ra;
            rb_list[i] = rb;

            Vec2 ra_perp(-ra.m_y, ra.m_x);
            Vec2 rb_perp(-rb.m_y, rb.m_x);

            Vec2 ang_linear_vel_a = ra_perp * ang_vel_a;
            Vec2 ang_linear_vel_b = rb_perp * ang_vel_b;

            Vec2 relative_vel = (vel_b + ang_linear_vel_b) - (vel_a + ang_linear_vel_a);
            Vec2 tangent = relative_vel - normal * relative_vel.dot(normal);

            if (PhysicsMath::equals(tangent, Vec2()))
                continue;

            tangent.normalize();

            float ra_perp_dot_t = ra_perp.dot(tangent);
            float rb_perp_dot_t = rb_perp.dot(tangent);

            float denom = inv_mass_a + inv_mass_b +
                          (ra_perp_dot_t * ra_perp_dot_t) * inv_rot_inertia_a +
                          (rb_perp_dot_t * rb_perp_dot_t) * inv_rot_inertia_b;

            float jt = -relative_vel.dot(tangent);
            jt /= denom;
            jt /= static_cast<float>(contact_count);

            Vec2 friction_impulse;
            float j = j_list[i];

            if (std::abs(jt) <= j * static_friction)
            {
                friction_impulse = tangent * jt;
            }
            else
            {
                friction_impulse = tangent * (-j * kinetic_friction);
            }

            friction_impulse_list[i] = friction_impulse;
        }

        for (int i = 0; i < contact_count; ++i)
        {
            Vec2 &friction_impulse = friction_impulse_list[i];
            Vec2 &ra = ra_list[i];
            Vec2 &rb = rb_list[i];

            vel_a -= friction_impulse * inv_mass_a;
            vel_b += friction_impulse * inv_mass_b;

            ang_vel_a -= ra.cross(friction_impulse) * inv_rot_inertia_a;
            ang_vel_b += rb.cross(friction_impulse) * inv_rot_inertia_b;
        }

        rb_a.m_vel = vel_a;
        rb_b.m_vel = vel_b;
        rb_a.m_angular_vel = ang_vel_a;
        rb_b.m_angular_vel = ang_vel_b;
    }
}
