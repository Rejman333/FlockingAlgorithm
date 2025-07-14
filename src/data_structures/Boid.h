#pragma once

#include <vector>
#include <random>

#include "raylib.h"
#include "raymath.h"


struct Boid {
    Vector2 position;
    Vector2 velocity_norm;
    float speed;
    Vector2 acceleration;

    int hash_table_id;
    int cluster_id = 0;
};

void apply_boid_behaviors(
    Boid &boid,
    const std::vector<std::pair<Boid *, float> > &neighbors,
    float sep_range_sqr, float sep_strength,
    float ali_range_sqr, float ali_strength,
    float coh_range_sqr, float coh_strength);

std::vector<Boid> fill_boids(size_t boids_number, int screenWidth, int screenHeight);
