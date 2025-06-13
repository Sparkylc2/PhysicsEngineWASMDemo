//
// Created by Lukas Campbell on 08/06/2025.
//
#pragma once

constexpr bool DEBUG = false;
constexpr uint16_t TOTAL_ITER = 1000;

extern float DT;
extern bool IS_PAUSED;
extern std::vector<std::vector<size_t>> COLLISION_PAIRS;
extern std::vector<Collisions::CollisionResult> COLLISION_RESULTS;
extern std::deque<Rigidbody> BODIES;
extern std::deque<Rigidbody*> WALL_BODIES;
