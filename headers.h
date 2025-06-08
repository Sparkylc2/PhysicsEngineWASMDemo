//
// Created by Lukas Campbell on 03/06/2025.
//

#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <cstdint>

#include <wasm_simd128.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef FLT_MAX
#define FLT_MAX std::numeric_limits<float>::max()
#endif

struct Vec2;
struct AABB;
struct Rigidbody;

#include "vec2.h"
#include "enums.h"
#include "aabb.h"
#include "physics_math.h"
#include "rigidbody.h"
