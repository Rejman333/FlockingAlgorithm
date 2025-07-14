#pragma once

#include <vector>
#include "raylib.h"
#include "Boid.h"
#include "hash_table.h"
#include "../tools/methods.h"
#include "QuadTree.h"


struct ClusterCenter {
    Vector2 position;
};

void k_means(std::vector<Boid> &boids, int k, int max_iterations);

void k_means(std::vector<Boid> &boids, int k, int max_iterations, HashTable *hash_table);

void k_means(std::vector<Boid> &boids, int k, int max_iterations, QuadTree<10> *quad_tree);

std::vector<Color> generate_random_colors(int k);
