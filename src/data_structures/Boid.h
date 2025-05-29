#pragma once

#include <vector>

#include "raylib.h"


struct Boid {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;

    int hash_table_id;
};

void separation(Boid &boid, const std::vector<std::pair<Boid *, float> > &neighbors_with_distance,
                float separation_range_squared, float separation_strength);

void alignment(Boid &boid,
               const std::vector<std::pair<Boid *, float> > &neighbors_with_distance,
               float alignment_range_squared, float alignment_strength);

void cohesion(Boid &boid,
              const std::vector<std::pair<Boid *, float> > &neighbors_with_distance,
              float cohesion_range_squared, float cohesion_strength);
