#pragma once

#include <vector>
#include <random>

#include "raylib.h"
#include "raymath.h"


struct Boid {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;

    int hash_table_id;
};

void apply_boid_behaviors();

std::vector<Boid> fill_boids(size_t boids_number, int screenWidth, int screenHeight);
