#pragma once

#include <vector>
#include "raylib.h"
#include "Boid.h"
#include "string"
#include "hash_table.h"
#include "QuadTree.h"

struct ClusterCenter {
    Vector2 position;
};

void k_means(std::vector<Boid>& boids, int k, std::string& method, HashTable* hash_table, QuadTree<10>* quad_tree,int max_iterations = 100);
std::vector<Color> generate_random_colors(int k);
