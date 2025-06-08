//
// Created by Lukas Campbell on 03/06/2025.
//

#pragma once
#include "headers.h"
namespace Collisions
{
    struct CollisionResult
    {
        bool m_is_colliding = false;
        Vec2 m_normal = {};
        float m_depth = 0.0;
        float m_distance_sq = 0.0;
        std::vector<Vec2> m_points_of_contact;
        int m_contact_count = 0;

        Rigidbody& m_rb_a = nullptr;
        Rigidbody& m_rb_b = nullptr;

        CollisionResult() = default;
        CollisionResult(Rigidbody& rb_a, Rigidbody& rb_b, Vec2 normal, float depth) {
            m_rb_a = rb_a;
            m_rb_b = rb_b;
            
        }
    };

    CollisionResult collide(const Rigidbody &rb_a, const Rigidbody &rb_b)
    {
        if (rb_a.m_shape_type == ShapeType::BOX)
        {
            if (rb_b.m_shape_type == ShapeType::BOX)
            {
                return intersect_polygon(rb_a, rb_b);
            }
            else
            {
                CollisionResult res = intersect_circle_polygon(rb_b, rb_a);
                res.m_normal *= -1;
                return res;
            }
        }

        if (rb_a.m_shape_type == ShapeType::CIRCLE)
        {
            if (rb_b.m_shape_type == ShapeType::BOX)
            {
                return intersect_circle_polygon(rb_a, rb_b);
            }
            else
            {
                return intersect_circle(rb_a, rb_b);
            }
        }

        return CollisionResult();
    }

    CollisionResult intersect_polygon(Rigidbody rb_a, Rigidbody rb_b)
    {
        float trans_verts_a_len = rb_a.m_transformed_verts.size();
        float trans_verts_b_len = rb_b.m_transformed_verts.size();

        float depth = __FLT_MAX__;
        Vec2 normal = {};

        for (size_t vert_idx_a = 0; vert_idx_a < trans_verts_a_len; ++vert_idx_a)
        {
            Vec2 trans_vert_a = rb_a.m_transformed_verts[vert_idx_a];
            Vec2 trans_vert_b = rb_a.m_transformed_verts[(vert_idx_a + 1) % trans_verts_a_len];

            Vec2 edge = trans_vert_b - trans_vert_a;
            Vec2 axis = {-edge.y, edge.x};
            axis.normalize();

            std::vector<float> min_max_a = project_vertices(rb_a.m_transformed_verts, axis);
            std::vector<float> min_max_b = project_vertices(rb_b.m_transformed_verts, axis);

            if (min_max_a[0] >= min_max_b[1] || min_max_b[0] >= min_max_a[1])
                return CollisionResult();

            float axis_depth = std::min(min_max_b[1] - min_max_a[0], min_max_a[1] - min_max_b[0]);

            if (axis_depth < depth)
            {
                depth = axis_depth;
                normal = axis;
            }
        }
        for (size_t vert_idx_b = 0; vert_idx_b < trans_verts_b_len; ++vert_idx_b)
        {
            Vec2 trans_vert_a = rb_b.m_transformed_verts[vert_idx_b];
            Vec2 trans_vert_b = rb_b.m_transformed_verts[(vert_idx_b + 1) % trans_verts_b_len];

            Vec2 edge = trans_vert_b - trans_vert_a;
            Vec2 axis = {-edge.y, edge.x};
            axis.normalize();

            std::vector<float> min_max_a = project_vertices(rb_a.m_transformed_verts, axis);
            std::vector<float> min_max_b = project_vertices(rb_b.m_transformed_verts, axis);

            if (min_max_a[0] >= min_max_b[1] || min_max_b[0] >= min_max_a[1])
                return CollisionResult();

            float axis_depth = std::min(min_max_b[1] - min_max_a[0], min_max_a[1] - min_max_b[0]);

            if (axis_depth < depth)
            {
                depth = axis_depth;
                normal = axis;
            }
        }

        if ((rb_b.m_pos - rb_a.m_pos).dot(normal) < 0) normal *= -1;

        return CollisionResult(rb_a, rb_b, true, normal, depth);
    }

    bool intersect_aabb(const Rigidbody &rb_a, const Rigidbody &rb_b)
    {
        return !(
            (rb_a.m_aabb.m_max.x <= rb_b.m_aabb.m_min.x) || (rb_b.m_aabb.m_max.x <= rb_a.m_aabb.m_min.x) || (rb_a.m_aabb.m_max.y <= rb_b.m_aabb.m_min.y) || (rb_b.m_aabb.m_max.y <= rb_a.m_aabb.m_min.y));
    }
}
