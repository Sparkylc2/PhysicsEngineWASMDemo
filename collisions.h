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
        float m_depth = 0.0f;
        float m_distance_sq = 0.0f;
        std::vector<Vec2> m_points_of_contact;
        int m_contact_count = 0;

        CollisionResult() = default;
        CollisionResult(bool is_colliding, Vec2 normal, float depth)
            : m_is_colliding(is_colliding), m_normal(normal), m_depth(depth) {}
        CollisionResult(float distance_sq, Vec2 point)
            : m_distance_sq(distance_sq)
        {
            m_points_of_contact.push_back(point);
        }
    };

    inline CollisionResult intersect_polygon(const Rigidbody &rb_a, const Rigidbody &rb_b);
    inline CollisionResult intersect_circle(const Rigidbody &rb_a, const Rigidbody &rb_b);
    inline CollisionResult intersect_circle_polygon(const Rigidbody &rb_a, const Rigidbody &rb_b);
    inline void find_collision_points(const Rigidbody &rb_a, const Rigidbody &rb_b, CollisionResult &result);

    inline std::vector<float> project_vertices(const std::vector<Vec2> &verts, const Vec2 &axis);
    inline std::vector<float> project_circle(const Vec2 &center, const Vec2 &axis, float radius);
    inline int find_closest_point_on_polygon(const Vec2 &circle_center, const std::vector<Vec2> &trans_verts);
    inline CollisionResult point_segment_distance(const Vec2 &point, const Vec2 &seg_start, const Vec2 &seg_end);

    inline std::vector<Vec2> find_circles_collision_point(const Vec2 &center_a, float radius_a,
                                                          const Vec2 &center_b, float radius_b);
    inline std::vector<Vec2> find_polygons_collision_points(const std::vector<Vec2> &trans_verts_a,
                                                            const std::vector<Vec2> &trans_verts_b);
    inline std::vector<Vec2> find_circle_polygon_collision_point(const Vec2 &circle_center, float circle_radius,
                                                                 const Vec2 &polygon_center,
                                                                 const std::vector<Vec2> &trans_verts);

    inline Vec2 calculate_line_intersection(const Vec2 &p1, const Vec2 &p2, const Vec2 &p3, const Vec2 &p4);
    inline bool is_point_in_line_segment(const Vec2 &point, const Vec2 &line_start, const Vec2 &line_end);

    inline CollisionResult collide(const Rigidbody &rb_a, const Rigidbody &rb_b)
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

    inline bool intersect_aabb(const AABB &aabb_a, const AABB &aabb_b)
    {
        return !(
            (aabb_a.m_max.m_x <= aabb_b.m_min.m_x) ||
            (aabb_b.m_max.m_x <= aabb_a.m_min.m_x) ||
            (aabb_a.m_max.m_y <= aabb_b.m_min.m_y) ||
            (aabb_b.m_max.m_y <= aabb_a.m_min.m_y));
    }

    inline bool intersect_aabb(const Rigidbody &rb_a, const Rigidbody &rb_b)
    {
        return intersect_aabb(rb_a.m_aabb, rb_b.m_aabb);
    }

    inline bool intersect_aabb_with_point(const AABB &aabb, const Vec2 &point)
    {
        return (point.m_x >= aabb.m_min.m_x && point.m_x <= aabb.m_max.m_x &&
                point.m_y >= aabb.m_min.m_y && point.m_y <= aabb.m_max.m_y);
    }

    inline CollisionResult intersect_circle(const Rigidbody &rb_a, const Rigidbody &rb_b)
    {
        Vec2 direction = rb_b.m_pos - rb_a.m_pos;
        float distance = direction.len();
        float radius_sum = rb_a.m_radius + rb_b.m_radius;

        if (distance < radius_sum)
        {
            direction.normalize();
            return CollisionResult(true, direction, radius_sum - distance);
        }

        return CollisionResult();
    }

    inline CollisionResult intersect_polygon(const Rigidbody &rb_a, const Rigidbody &rb_b)
    {
        size_t trans_verts_a_len = rb_a.m_transformed_verts.size();
        size_t trans_verts_b_len = rb_b.m_transformed_verts.size();

        float depth = FLT_MAX;
        Vec2 normal = {};

        for (size_t vert_idx_a = 0; vert_idx_a < trans_verts_a_len; ++vert_idx_a)
        {
            Vec2 trans_vert_a = rb_a.m_transformed_verts[vert_idx_a];
            Vec2 trans_vert_b = rb_a.m_transformed_verts[(vert_idx_a + 1) % trans_verts_a_len];

            Vec2 edge = trans_vert_b - trans_vert_a;
            Vec2 axis(-edge.m_y, edge.m_x);
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
            Vec2 axis(-edge.m_y, edge.m_x);
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

        if ((rb_b.m_pos - rb_a.m_pos).dot(normal) < 0)
            normal *= -1;

        return CollisionResult(true, normal, depth);
    }

    inline CollisionResult intersect_circle_polygon(const Rigidbody &circle_rb, const Rigidbody &polygon_rb)
    {
        float depth = FLT_MAX;
        Vec2 normal = {};

        const std::vector<Vec2> &trans_verts = polygon_rb.m_transformed_verts;
        size_t vert_count = trans_verts.size();

        for (size_t vert_idx = 0; vert_idx < vert_count; ++vert_idx)
        {
            Vec2 edge = trans_verts[(vert_idx + 1) % vert_count] - trans_verts[vert_idx];
            Vec2 axis(-edge.m_y, edge.m_x);
            axis.normalize();

            std::vector<float> min_max_a = project_vertices(trans_verts, axis);
            std::vector<float> min_max_b = project_circle(circle_rb.m_pos, axis, circle_rb.m_radius);

            if (min_max_a[0] >= min_max_b[1] || min_max_b[0] >= min_max_a[1])
                return CollisionResult();

            float axis_depth = std::min(min_max_b[1] - min_max_a[0], min_max_a[1] - min_max_b[0]);

            if (axis_depth < depth)
            {
                depth = axis_depth;
                normal = axis;
            }
        }

        int closest_idx = find_closest_point_on_polygon(circle_rb.m_pos, trans_verts);
        if (closest_idx == -1)
            return CollisionResult();

        Vec2 axis = (trans_verts[closest_idx] - circle_rb.m_pos);
        axis.normalize();

        std::vector<float> min_max_a = project_vertices(trans_verts, axis);
        std::vector<float> min_max_b = project_circle(circle_rb.m_pos, axis, circle_rb.m_radius);

        if (min_max_a[0] >= min_max_b[1] || min_max_b[0] >= min_max_a[1])
            return CollisionResult();

        float axis_depth = std::min(min_max_b[1] - min_max_a[0], min_max_a[1] - min_max_b[0]);

        if (axis_depth < depth)
        {
            depth = axis_depth;
            normal = axis;
        }

        if ((polygon_rb.m_pos - circle_rb.m_pos).dot(normal) < 0)
            normal *= -1;

        return CollisionResult(true, normal, depth);
    }

    inline void find_collision_points(const Rigidbody &rb_a, const Rigidbody &rb_b, CollisionResult &result)
    {
        if (rb_a.m_shape_type == ShapeType::BOX)
        {
            if (rb_b.m_shape_type == ShapeType::BOX)
            {
                result.m_points_of_contact = find_polygons_collision_points(
                    rb_a.m_transformed_verts, rb_b.m_transformed_verts);
                result.m_contact_count = result.m_points_of_contact.size();
            }
            else if (rb_b.m_shape_type == ShapeType::CIRCLE)
            {
                result.m_points_of_contact = find_circle_polygon_collision_point(
                    rb_b.m_pos, rb_b.m_radius, rb_a.m_pos, rb_a.m_transformed_verts);
                result.m_contact_count = 1;
            }
        }
        else if (rb_a.m_shape_type == ShapeType::CIRCLE)
        {
            if (rb_b.m_shape_type == ShapeType::BOX)
            {
                result.m_points_of_contact = find_circle_polygon_collision_point(
                    rb_a.m_pos, rb_a.m_radius, rb_b.m_pos, rb_b.m_transformed_verts);
                result.m_contact_count = 1;
            }
            else if (rb_b.m_shape_type == ShapeType::CIRCLE)
            {
                result.m_points_of_contact = find_circles_collision_point(
                    rb_a.m_pos, rb_a.m_radius, rb_b.m_pos, rb_b.m_radius);
                result.m_contact_count = 1;
            }
        }
    }

    inline std::vector<float> project_vertices(const std::vector<Vec2> &verts, const Vec2 &axis)
    {
        float min = FLT_MAX;
        float max = -FLT_MAX;

        for (const Vec2 &vert : verts)
        {
            float projected = vert.dot(axis);
            if (projected < min)
                min = projected;
            if (projected > max)
                max = projected;
        }

        return {min, max};
    }

    inline std::vector<float> project_circle(const Vec2 &center, const Vec2 &axis, float radius)
    {
        Vec2 direction_and_radius = axis * radius;
        float p1 = (center + direction_and_radius).dot(axis);
        float p2 = (center - direction_and_radius).dot(axis);

        if (p1 > p2)
            std::swap(p1, p2);

        return {p1, p2};
    }

    inline int find_closest_point_on_polygon(const Vec2 &circle_center, const std::vector<Vec2> &trans_verts)
    {
        int result = -1;
        float min_dist_sq = FLT_MAX;

        if (trans_verts.empty())
            return result;

        for (size_t vert_idx = 0; vert_idx < trans_verts.size(); ++vert_idx)
        {
            Vec2 diff = circle_center - trans_verts[vert_idx];
            float dist_sq = diff.dot(diff);

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                result = vert_idx;
            }
        }

        return result;
    }

    inline CollisionResult point_segment_distance(const Vec2 &point, const Vec2 &seg_start, const Vec2 &seg_end)
    {
        Vec2 line_seg = seg_end - seg_start;
        Vec2 point_to_start = point - seg_start;

        float projection = point_to_start.dot(line_seg);
        float line_seg_len_sq = line_seg.dot(line_seg);

        float t = projection / line_seg_len_sq;

        Vec2 closest_point;

        if (t <= 0.0f)
            closest_point = seg_start;
        else if (t >= 1.0f)
            closest_point = seg_end;
        else
            closest_point = seg_start + line_seg * t;

        Vec2 diff = point - closest_point;
        float dist_sq = diff.dot(diff);

        return CollisionResult(dist_sq, closest_point);
    }

    inline std::vector<Vec2> find_circles_collision_point(const Vec2 &center_a, float radius_a,
                                                          const Vec2 &center_b, float radius_b)
    {
        Vec2 dir = center_b - center_a;
        dir.normalize();
        return {center_a + dir * radius_a};
    }

    inline std::vector<Vec2> find_polygons_collision_points(const std::vector<Vec2> &trans_verts_a,
                                                            const std::vector<Vec2> &trans_verts_b)
    {
        Vec2 contact_a, contact_b;
        int contact_count = 0;
        float min_dist_sq = FLT_MAX;

        for (size_t i = 0; i < trans_verts_a.size(); ++i)
        {
            const Vec2 &point = trans_verts_a[i];

            for (size_t j = 0; j < trans_verts_b.size(); ++j)
            {
                CollisionResult result = point_segment_distance(
                    point, trans_verts_b[j], trans_verts_b[(j + 1) % trans_verts_b.size()]);

                if (PhysicsMath::equals(result.m_distance_sq, min_dist_sq))
                {
                    if (!PhysicsMath::equals(result.m_points_of_contact[0], contact_a))
                    {
                        contact_b = result.m_points_of_contact[0];
                        contact_count = 2;
                    }
                }
                else if (result.m_distance_sq < min_dist_sq)
                {
                    min_dist_sq = result.m_distance_sq;
                    contact_a = result.m_points_of_contact[0];
                    contact_count = 1;
                }
            }
        }

        for (size_t i = 0; i < trans_verts_b.size(); ++i)
        {
            const Vec2 &point = trans_verts_b[i];

            for (size_t j = 0; j < trans_verts_a.size(); ++j)
            {
                CollisionResult result = point_segment_distance(
                    point, trans_verts_a[j], trans_verts_a[(j + 1) % trans_verts_a.size()]);

                if (PhysicsMath::equals(result.m_distance_sq, min_dist_sq))
                {
                    if (!PhysicsMath::equals(result.m_points_of_contact[0], contact_a))
                    {
                        contact_b = result.m_points_of_contact[0];
                        contact_count = 2;
                    }
                }
                else if (result.m_distance_sq < min_dist_sq)
                {
                    min_dist_sq = result.m_distance_sq;
                    contact_a = result.m_points_of_contact[0];
                    contact_count = 1;
                }
            }
        }

        if (contact_count == 1)
            return {contact_a};
        else
            return {contact_a, contact_b};
    }

    inline std::vector<Vec2> find_circle_polygon_collision_point(const Vec2 &circle_center, float circle_radius,
                                                                 const Vec2 &polygon_center,
                                                                 const std::vector<Vec2> &trans_verts)
    {
        float min_dist_sq = FLT_MAX;
        Vec2 contact_point;

        for (size_t i = 0; i < trans_verts.size(); ++i)
        {
            CollisionResult result = point_segment_distance(
                circle_center, trans_verts[i], trans_verts[(i + 1) % trans_verts.size()]);

            if (result.m_distance_sq < min_dist_sq)
            {
                min_dist_sq = result.m_distance_sq;
                contact_point = result.m_points_of_contact[0];
            }
        }

        return {contact_point};
    }

    inline Vec2 calculate_line_intersection(const Vec2 &p1, const Vec2 &p2, const Vec2 &p3, const Vec2 &p4)
    {
        float denominator = (p1.m_x - p2.m_x) * (p3.m_y - p4.m_y) - (p1.m_y - p2.m_y) * (p3.m_x - p4.m_x);

        if (denominator == 0)
            return Vec2();

        float intersect_x = ((p1.m_x * p2.m_y - p1.m_y * p2.m_x) * (p3.m_x - p4.m_x) -
                             (p1.m_x - p2.m_x) * (p3.m_x * p4.m_y - p3.m_y * p4.m_x)) /
                            denominator;
        float intersect_y = ((p1.m_x * p2.m_y - p1.m_y * p2.m_x) * (p3.m_y - p4.m_y) -
                             (p1.m_y - p2.m_y) * (p3.m_x * p4.m_y - p3.m_y * p4.m_x)) /
                            denominator;

        return Vec2(intersect_x, intersect_y);
    }

    inline bool is_point_in_line_segment(const Vec2 &point, const Vec2 &line_start, const Vec2 &line_end)
    {
        float line_length = (line_end - line_start).len();
        float total_dist = (point - line_start).len() + (point - line_end).len();

        const float epsilon = 0.0001f;
        return std::abs(total_dist - line_length) < epsilon;
    }
}
