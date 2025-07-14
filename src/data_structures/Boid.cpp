#include "Boid.h"

void apply_boid_behaviors(
    Boid &boid,
    const std::vector<std::pair<Boid *, float> > &neighbors,
    float sep_range_sqr, float sep_strength,
    float ali_range_sqr, float ali_strength,
    float coh_range_sqr, float coh_strength) {
    Vector2 sep_force = {}, ali_force = {}, coh_center = {};
    int sep_count = 0, ali_count = 0, coh_count = 0;

    for (int i = 0; i < neighbors.size(); ++i) {
        const Boid *other = neighbors[i].first;
        const float dist_sqr = neighbors[i].second;

        Vector2 diff = Vector2Subtract(boid.position, other->position);

        if (dist_sqr < sep_range_sqr && dist_sqr > 0.01f) {
            float inv_dist = 1.0f / sqrtf(dist_sqr); // większa siła przy bliskich boidach
            Vector2 diff = Vector2Subtract(boid.position, other->position);
            diff = Vector2Scale(Vector2Normalize(diff), inv_dist);
            sep_force = Vector2Add(sep_force, diff);
            sep_count++;
        }

        if (dist_sqr < ali_range_sqr) {
            ali_force = Vector2Add(ali_force, Vector2Scale(other->velocity_norm, other->speed));
            ali_count++;
        }

        if (dist_sqr < coh_range_sqr) {
            coh_center = Vector2Add(coh_center, other->position);
            coh_count++;
        }
    }

    if (sep_count > 0) {
        sep_force = Vector2Scale(Vector2Normalize(sep_force), sep_strength);
        boid.acceleration = Vector2Add(boid.acceleration, sep_force);
    }

    if (ali_count > 0) {
        ali_force = Vector2Scale(ali_force, 1.0f / ali_count);
        ali_force = Vector2Scale(Vector2Normalize(ali_force), ali_strength);
        boid.acceleration = Vector2Add(boid.acceleration, ali_force);
    }

    if (coh_count > 0) {
        coh_center = Vector2Scale(coh_center, 1.0f / coh_count);
        Vector2 to_center = Vector2Subtract(coh_center, boid.position);
        to_center = Vector2Scale(Vector2Normalize(to_center), coh_strength);
        boid.acceleration = Vector2Add(boid.acceleration, to_center);
    }
}

std::vector<Boid> fill_boids(const size_t boids_number,const int screenWidth, const int screenHeight) {
    std::vector<Boid> boids(boids_number);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution distribution_x(0.0f, static_cast<float>(screenWidth));
    std::uniform_real_distribution distribution_y(0.0f, static_cast<float>(screenHeight));
    std::uniform_real_distribution distribution(1.f, 5.0f);
    std::uniform_real_distribution distribution_normalize(-1.f, 1.f);

    for (auto &boid: boids) {
        boid.position = {distribution_x(gen), distribution_y(gen)};
        boid.velocity_norm = {distribution_normalize(gen), distribution_normalize(gen)};
        boid.speed = distribution(gen);
        boid.acceleration = {0.0f, 0.0f};
    }

    return boids;
}