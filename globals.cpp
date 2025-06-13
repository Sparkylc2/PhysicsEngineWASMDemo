//
// Created by Lukas Campbell on 08/06/2025.
//
#include "headers.h"

float DT = 0.01f;
bool IS_PAUSED = false;
std::vector<std::vector<size_t>> COLLISION_PAIRS;
std::vector<Collisions::CollisionResult> COLLISION_RESULTS;
std::deque<Rigidbody> BODIES;

std::deque<Rigidbody*> WALL_BODIES;
