#include "Boid.h"
#include "raymath.h"

void separation(Boid &boid, const std::vector<std::pair<Boid *, float> > &neighbors_with_distance,
                float separation_range_squared, float separation_strength) {
    Vector2 separation_force = {0.0f, 0.0f};
    int separation_count = 0;
    for (auto [neighbor_boid, dist]: neighbors_with_distance) {
        if (dist < separation_range_squared && dist > 0.0f) {
            Vector2 diff = Vector2Subtract(boid.position, neighbor_boid->position);
            diff = Vector2Normalize(diff);
            diff = Vector2Scale(diff, 1.0f / dist);
            separation_force = Vector2Add(separation_force, diff);
            separation_count++;
        }
    }

    if (separation_count > 0) {
        separation_force = Vector2Scale(separation_force, 1.0f / static_cast<float>(separation_count));
        separation_force = Vector2Normalize(separation_force);
        separation_force = Vector2Scale(separation_force, separation_strength);
        boid.acceleration = Vector2Add(boid.acceleration, separation_force);
    }
}

void alignment(Boid &boid,
               const std::vector<std::pair<Boid *, float> > &neighbors_with_distance,
               float alignment_range_squared, float alignment_strength) {
    Vector2 alignment_force = {0.0f, 0.0f};
    int alignment_count = 0;

    for (auto [neighbor_boid, dist]: neighbors_with_distance) {
        if (dist < alignment_range_squared && dist > 0.0f) {
            alignment_force = Vector2Add(alignment_force, neighbor_boid->velocity);
            alignment_count++;
        }
    }

    if (alignment_count > 0) {
        alignment_force = Vector2Scale(alignment_force, 1.0f / static_cast<float>(alignment_count));
        alignment_force = Vector2Normalize(alignment_force);
        alignment_force = Vector2Scale(alignment_force, alignment_strength);
        boid.acceleration = Vector2Add(boid.acceleration, alignment_force);
    }
}

void cohesion(Boid &boid,
              const std::vector<std::pair<Boid *, float> > &neighbors_with_distance,
              float cohesion_range_squared, float cohesion_strength) {
    Vector2 center_of_mass = {0.0f, 0.0f};
    int cohesion_count = 0;

    for (auto [neighbor_boid, dist]: neighbors_with_distance) {
        if (dist < cohesion_range_squared && dist > 0.0f) {
            center_of_mass = Vector2Add(center_of_mass, neighbor_boid->position);
            cohesion_count++;
        }
    }

    if (cohesion_count > 0) {
        center_of_mass = Vector2Scale(center_of_mass, 1.0f / static_cast<float>(cohesion_count));
        Vector2 direction_to_center = Vector2Subtract(center_of_mass, boid.position);
        direction_to_center = Vector2Normalize(direction_to_center);
        direction_to_center = Vector2Scale(direction_to_center, cohesion_strength);
        boid.acceleration = Vector2Add(boid.acceleration, direction_to_center);
    }
}
