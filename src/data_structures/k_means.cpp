#include "k_means.h"

std::vector<Color> generate_random_colors(int k) {
    std::vector<Color> colors;
    for (int i = 0; i < k; ++i) {
        Color c = {
            static_cast<unsigned char>(rand() % 128 + 127),
            static_cast<unsigned char>(rand() % 128 + 127),
            static_cast<unsigned char>(rand() % 128 + 127),
            255
        };
        colors.push_back(c);
    }
    return colors;
}

void k_means(std::vector<Boid> &boids, int k, int max_iterations) {
    if (boids.empty() || k <= 0) return;

    int n = boids.size();
    std::vector<Vector2> centroids(k);
    for (int i = 0; i < k; ++i)
        centroids[i] = boids[rand() % n].position;

    std::vector<Vector2> new_centroids(k);
    std::vector<int> cluster_sizes(k);
    bool changed = true;
    int iterations = 0;

    while (changed && iterations++ < max_iterations) {
        changed = false;
        std::fill(new_centroids.begin(), new_centroids.end(), Vector2{0, 0});
        std::fill(cluster_sizes.begin(), cluster_sizes.end(), 0);

        for (auto &boid: boids) {
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = 0;

            for (int i = 0; i < k; ++i) {
                float dist = Vector2DistanceSqr(boid.position, centroids[i]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = i;
                }
            }

            if (boid.cluster_id != best_cluster) {
                boid.cluster_id = best_cluster;
                changed = true;
            }

            new_centroids[boid.cluster_id] = Vector2Add(new_centroids[boid.cluster_id], boid.position);
            cluster_sizes[boid.cluster_id]++;
        }

        for (int i = 0; i < k; ++i) {
            if (cluster_sizes[i] > 0)
                centroids[i] = Vector2Scale(new_centroids[i], 1.0f / cluster_sizes[i]);
        }
    }
}

void k_means(std::vector<Boid> &boids, int k, int max_iterations, HashTable *hash_table) {
    int n = boids.size();
    std::vector<Vector2> centroids(k);
    for (int i = 0; i < k; ++i)
        centroids[i] = boids[rand() % n].position;

    bool changed = true;
    int iterations = 0;

    while (changed && iterations++ < max_iterations) {
        changed = false;

        for (auto &boid: boids) {
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = 0;

            for (int i = 0; i < k; ++i) {
                float dist = Vector2DistanceSqr(boid.position, centroids[i]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = i;
                }
            }

            if (boid.cluster_id != best_cluster) {
                boid.cluster_id = best_cluster;
                changed = true;
            }
        }

        for (int cluster_id = 0; cluster_id < k; ++cluster_id) {
            Vector2 center = centroids[cluster_id];
            int cell_id = hash_table->get_cell_id(center);

            std::vector<Boid *> nearby_boids = hash_table->get_boids_in_range(cell_id);
            Vector2 sum = {0, 0};
            int count = 0;

            for (Boid *boid: nearby_boids) {
                if (boid == nullptr) continue;
                if (boid->cluster_id == cluster_id) {
                    sum = Vector2Add(sum, boid->position);
                    count++;
                }
            }

            if (count > 0)
                centroids[cluster_id] = Vector2Scale(sum, 1.0f / count);
        }
    }
}

void k_means(std::vector<Boid> &boids, int k, int max_iterations, QuadTree<10> *quad_tree) {
    int n = boids.size();
    std::vector<Vector2> centroids(k);
    for (int i = 0; i < k; ++i)
        centroids[i] = boids[rand() % n].position;

    bool changed = true;
    int iterations = 0;

    while (changed && iterations++ < max_iterations) {
        changed = false;

        for (auto &boid: boids) {
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = 0;

            for (int i = 0; i < k; ++i) {
                float dist = Vector2DistanceSqr(boid.position, centroids[i]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = i;
                }
            }

            if (boid.cluster_id != best_cluster) {
                boid.cluster_id = best_cluster;
                changed = true;
            }
        }

        // DO NOT reset or rebuild quad_tree here — done externally!

        for (int cluster_id = 0; cluster_id < k; ++cluster_id) {
            Vector2 center = centroids[cluster_id];
            const float search_radius = 50.0f;

            std::vector<Boid *> nearby_boids = quad_tree->query(center, search_radius);
            Vector2 sum = {0, 0};
            int count = 0;

            for (Boid *boid: nearby_boids) {
                if (boid == nullptr) continue;
                if (boid->cluster_id == cluster_id) {
                    sum = Vector2Add(sum, boid->position);
                    count++;
                }
            }

            if (count > 0)
                centroids[cluster_id] = Vector2Scale(sum, 1.0f / count);
        }
    }
}
