//
// Created by Lukas Campbell on 08/06/2025.
//

#pragma once
#include "headers.h"

struct Force
{
    virtual Vec2 get_force(Rigidbody *r, const Vec2 &pos) = 0;
    virtual Vec2 get_application_point(Rigidbody *r, const Vec2 &pos) = 0;

    virtual ~Force() = default;
};

struct Spring : public Force
{
    Rigidbody *m_rb_a = nullptr;
    Rigidbody *m_rb_b = nullptr;

    float m_equilibrium_len = 1.0f;
    float m_stiffness = 100.0f;

    float m_damping = 0.0f;

    Vec2 m_loc_anchor_a = Vec2();
    Vec2 m_loc_anchor_b = Vec2();
    Vec2 m_anchor = Vec2();

    bool m_two_b_spring = true;

    Spring() = default;

    // Spring(Rigidbody *r, Vec2 loc_anchor_a, Vec2 anchor)
    //     : m_rb_a(r), m_loc_anchor_a(loc_anchor_a), m_anchor(anchor)
    // {
    //     Vec2 anchor_a = PhysicsMath::transform(loc_anchor_a, r->m_pos, r->m_angle);
    //     m_equilibrium_len = (anchor_a - m_anchor).len();
    //     m_two_b_spring = false;
    // }

    // Spring(Rigidbody *r, Vec2 loc_anchor_a, Vec2 anchor, float equilibrium_len, float stiffness, float damping)
    //     : m_rb_a(r), m_loc_anchor_a(loc_anchor_a), m_anchor(anchor), m_equilibrium_len(equilibrium_len), m_stiffness(stiffness), m_damping(damping)
    // {
    //     Vec2 anchor_a = PhysicsMath::transform(loc_anchor_a, r->m_pos, r->m_angle);
    //     m_two_b_spring = false;
    // }

    // Spring(Rigidbody *r, Vec2 loc_anchor_a, Vec2 anchor, float stiffness, float damping)
    //     : m_rb_a(r), m_loc_anchor_a(loc_anchor_a), m_anchor(anchor), m_stiffness(stiffness), m_damping(damping)
    // {
    //     Vec2 anchor_a = PhysicsMath::transform(loc_anchor_a, r->m_pos, r->m_angle);
    //     m_equilibrium_len = (anchor_a - m_anchor).len();
    //     m_two_b_spring = false;
    // }

    // Spring(Rigidbody *r_a, Rigidbody *r_b, Vec2 anchor_a, Vec2 anchor_b)
    //     : m_rb_a(r_a), m_rb_b(r_b)
    // {
    //     m_loc_anchor_a = PhysicsMath::transform(anchor_a - r_a->m_pos, Vec2(), -r_a->m_angle);
    //     m_loc_anchor_b = PhysicsMath::transform(anchor_b - r_b->m_pos, Vec2(), -r_b->m_angle);
    //     m_two_b_spring = true;
    //     m_equilibrium_len = (anchor_b - anchor_a).len();
    // }

    // Spring(Rigidbody *r_a, Rigidbody *r_b, Vec2 anchor_a, Vec2 anchor_b, float stiffness, float damping)
    //     : m_rb_a(r_a), m_rb_b(r_b), m_stiffness(stiffness), m_damping(damping)
    // {
    //     m_loc_anchor_a = PhysicsMath::transform(anchor_a - r_a->m_pos, Vec2(), -r_a->m_angle);
    //     m_loc_anchor_b = PhysicsMath::transform(anchor_b - r_b->m_pos, Vec2(), -r_b->m_angle);
    //     m_two_b_spring = true;
    //     m_equilibrium_len = (anchor_b - anchor_a).len();
    // }

    Vec2 get_force(Rigidbody *r, const Vec2 &pos) override
    {
        Vec2 parent_w_anchor = Vec2();
        Vec2 other_w_anchor = Vec2();

        Vec2 parent_vel = Vec2();
        Vec2 other_vel = Vec2();

        if (m_two_b_spring)
        {
            if (r == m_rb_a)
            {
                parent_w_anchor = PhysicsMath::transform(m_loc_anchor_a, pos, m_rb_a->m_angle);
                other_w_anchor = PhysicsMath::transform(m_loc_anchor_b, m_rb_b->m_pos, m_rb_b->m_angle);

                parent_vel = m_rb_a->m_vel;
                other_vel = m_rb_b->m_vel;
            }
            else
            {
                parent_w_anchor = PhysicsMath::transform(m_loc_anchor_b, pos, m_rb_b->m_angle);
                other_w_anchor = PhysicsMath::transform(m_loc_anchor_a, m_rb_a->m_pos, m_rb_a->m_angle);

                parent_vel = m_rb_b->m_vel;
                other_vel = m_rb_a->m_vel;
            }
        }
        else
        {
            parent_w_anchor = PhysicsMath::transform(m_loc_anchor_a, pos, m_rb_a->m_angle);
            other_w_anchor = m_anchor;

            parent_vel = m_rb_a->m_vel;
        }

        Vec2 dir = other_w_anchor - parent_w_anchor;
        float displacement = dir.len();
        dir.normalize();

        float f = (displacement - m_equilibrium_len) * m_stiffness;
        f += dir.dot(other_vel - parent_vel) * m_damping;

        return dir * f;
    }

    Vec2 get_application_point(Rigidbody *r, const Vec2 &pos) override
    {
        if (r == m_rb_a)
        {
            return PhysicsMath::transform(m_loc_anchor_a, pos, r->m_angle);
        }
        else
        {
            return PhysicsMath::transform(m_loc_anchor_b, pos, r->m_angle);
        }
    }
};

struct Motor : public Force
{

    Rigidbody *m_r = nullptr;
    float m_target_ang_vel = 5.0f;

    Motor(Rigidbody *r) : m_r(r) {};

    Motor(Rigidbody *r, float target_ang_vel) : m_r(r), m_target_ang_vel(target_ang_vel) {};

    Vec2 get_force(Rigidbody *r, const Vec2 &pos)
    {
        m_r->m_angular_vel = m_target_ang_vel;
        return Vec2();
    }

    Vec2 get_application_point(Rigidbody *r, const Vec2 &pos)
    {
        return PhysicsMath::transform(Vec2(), pos, r->m_angle);
    }
};

struct Gravity : public Force
{
    Vec2 m_gravity = Vec2(0.0f, 9.81f);

    Gravity() = default;

    Gravity(const Vec2 &gravity) : m_gravity(gravity) {};

    Vec2 get_force(Rigidbody *r, const Vec2 &pos) override
    {
        return m_gravity * r->m_mass;
    }

    Vec2 get_application_point(Rigidbody *r, const Vec2 &pos) override
    {
        return pos;
    }
};
