

#ifndef K_MEANS_H
#define K_MEANS_H
#include <vector>
#include "raylib.h"
#include "Boid.h"

struct ClusterCenter {
    Vector2 position;
};

void run_kmeans(std::vector<Boid>& boids, int k, std::vector<int>& assignments);
std::vector<Color> generate_random_colors(int k);


#endif //K_MEANS_H
