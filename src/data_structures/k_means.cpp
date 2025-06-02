//
// Created by mrszy on 01.06.2025.
//

#include "k_means.h"

std::vector<Color> generate_random_colors(int k) {
    std::vector<Color> colors;
    for (int i = 0; i < k; ++i) {
        Color c = { static_cast<unsigned char>(rand() % 128 + 127),
                    static_cast<unsigned char>(rand() % 128 + 127),
                    static_cast<unsigned char>(rand() % 128 + 127),
                    255 };
        colors.push_back(c);
    }
    return colors;
}

static float distance_sqr(const Vector2& a, const Vector2& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

void run_kmeans(std::vector<Boid*>& boids, int k, std::vector<int>& assignments) {
    const int max_iterations = 100;
    const float epsilon = 0.001f;

    std::vector<ClusterCenter> centers(k);
    assignments.resize(boids.size(), -1);

    // Initialize cluster centers randomly
    for (int i = 0; i < k; ++i) {
        int idx = rand() % boids.size();
        centers[i].position = boids[idx]->position;
    }

    for (int iter = 0; iter < max_iterations; ++iter) {
        bool changed = false;

        // Assignment step
        for (size_t i = 0; i < boids.size(); ++i) {
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = -1;

            for (int c = 0; c < k; ++c) {
                float dist = distance_sqr(boids[i]->position, centers[c].position);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = c;
                }
            }

            if (assignments[i] != best_cluster) {
                changed = true;
                assignments[i] = best_cluster;
            }
        }

        // Update step
        std::vector<Vector2> new_centers(k, {0, 0});
        std::vector<int> counts(k, 0);

        for (size_t i = 0; i < boids.size(); ++i) {
            int cluster = assignments[i];
            new_centers[cluster].x += boids[i]->position.x;
            new_centers[cluster].y += boids[i]->position.y;
            counts[cluster]++;
        }

        for (int c = 0; c < k; ++c) {
            if (counts[c] > 0) {
                new_centers[c].x /= counts[c];
                new_centers[c].y /= counts[c];
            }
        }

        // Check for convergence
        float max_shift = 0.0f;
        for (int c = 0; c < k; ++c) {
            float shift = distance_sqr(centers[c].position, new_centers[c]);
            if (shift > max_shift) max_shift = shift;
            centers[c].position = new_centers[c];
        }

        if (!changed || max_shift < epsilon * epsilon) {
            break;
        }
    }
}