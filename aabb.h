//
// Created by Lukas Campbell on 03/06/2025.
//

#pragma once
#include "headers.h"
struct AABB
{
    Vec2 m_min = {};
    Vec2 m_max = {};

    AABB() = default;

    AABB(Vec2 min, Vec2 max)
    {
        m_min = min;
        m_max = max;
    }
};
