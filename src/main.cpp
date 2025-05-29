#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <random>

#include "raylib.h"
#include "raymath.h"
#include "data_structures/hash_table.h"
#include "data_structures/Boid.h"

#define NUMBER_OF_BOIDS 2000
#define BOID_RADIUS 2
#define CELL_SIZE 50
#define MAX_NEIGHBORS 20


void fill_boids(std::array<Boid, NUMBER_OF_BOIDS> &boids, const int screenWidth, const int screenHeight,
                HashTable &hash_table) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution distribution_x(0.0f, static_cast<float>(screenWidth));
    std::uniform_real_distribution distribution_y(0.0f, static_cast<float>(screenHeight));
    std::uniform_real_distribution distribution(-5.0f, 5.0f);

    int boid_index = 0;
    for (auto &boid: boids) {
        boid.position = {distribution_x(gen), distribution_y(gen)};
        boid.velocity = {distribution(gen), distribution(gen)};
        boid.acceleration = {.0f, 0.f};

        boid.hash_table_id = hash_table.put(boid.position, &boid);
        boid_index++;
    }
}


Vector2 wrap_position(Vector2 pos, const float screenWidth, const float screenHeight) {
    if (pos.x < 0) pos.x = screenWidth;
    else if (pos.x > screenWidth) pos.x = 0;

    if (pos.y < 0) pos.y = screenHeight;
    else if (pos.y > screenHeight) pos.y = 0;

    return pos;
}


int main() {
    // Initialization
    //--------------------------------------------------------------------------------------
    bool is_debug = false;

    constexpr int screenWidth = 1200;
    constexpr int screenHeight = 800;

    auto hash_table = HashTable(screenWidth, screenHeight,CELL_SIZE);
    std::array<Boid, NUMBER_OF_BOIDS> boids{};
    fill_boids(boids, screenWidth, screenHeight, hash_table);


    constexpr float separation_range = 15.0f;
    constexpr float alignment_range = 50.0f;
    constexpr float cohesion_range = 60.0f;

    constexpr float separation_range_squared = separation_range * separation_range;
    constexpr float alignment_range_squared = alignment_range * alignment_range;
    constexpr float cohesion_range_squared = cohesion_range * cohesion_range;

    const float separation_strength = 1.3f;
    const float alignment_strength = 0.6f;
    const float cohesion_strength = 0.8f;

    const float max_velocity = 3.5f;
    const float max_force = 0.10f;

    const float fov_angle_radians = DEG2RAD * 60.0f;
    const int scan_range_in_cells = std::max(static_cast<int>(alignment_range) / CELL_SIZE, 1);

    std::random_device rd;
    std::mt19937 rng(rd());

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_ALWAYS_RUN);
    InitWindow(screenWidth, screenHeight, "Boids");
    SetTargetFPS(60); // Set our game to run at 60 frames-per-second


    // Main game loop
    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        int i = 0;;
        for (auto &boid: boids) {
            std::vector<Boid *> boids_in_range = hash_table.get_boids_in_range(boid.hash_table_id, scan_range_in_cells);
            std::ranges::shuffle(boids_in_range, rng);
            std::array<std::pair<Boid *, float>, MAX_NEIGHBORS> neighbors;
            int neighbor_count = 0;
            for (auto boid_in_range: boids_in_range) {
                if (boid_in_range == &boid) continue;

                float dist_sqr = Vector2DistanceSqr(boid.position, boid_in_range->position);
                if (dist_sqr > cohesion_range_squared) continue;

                Vector2 diff = Vector2Subtract(boid_in_range->position, boid.position);
                float angle = Vector2Angle(boid.velocity, diff);
                if (angle > fov_angle_radians) continue;

                // float dot = Vector2DotProduct(Vector2Normalize(boid.velocity), Vector2Normalize(to_other));
                // if (dot < cos(fov_angle)) continue;

                neighbors[neighbor_count++] = {boid_in_range, dist_sqr};
                if (neighbor_count > MAX_NEIGHBORS - 1) break;
            }

            apply_boid_behaviors(boid, neighbors, neighbor_count,
                                 separation_range_squared, separation_strength,
                                 alignment_range_squared, alignment_strength,
                                 cohesion_range_squared, cohesion_strength);
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 m_pos = GetMousePosition();
            for (auto &boid: boids) {
                if (Vector2DistanceSqr(boid.position, m_pos) < 40000.f) {
                    Vector2 direction_to_center = Vector2Subtract(m_pos, boid.position);
                    boid.acceleration = Vector2Add(boid.acceleration, direction_to_center);
                }
            }
        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 m_pos = GetMousePosition();
            for (auto &boid: boids) {
                if (Vector2DistanceSqr(boid.position, m_pos) < 40000.f) {
                    Vector2 direction_to_center = Vector2Subtract(m_pos, boid.position);
                    boid.acceleration = Vector2Subtract(boid.acceleration, direction_to_center);
                }
            }
        }

        // teleports to other side, and adjust velocity]
        hash_table.reset();
        for (auto &boid: boids) {
            if (Vector2Length(boid.acceleration) > max_force) {
                boid.acceleration = Vector2Scale(Vector2Normalize(boid.acceleration), max_force);
            }

            boid.velocity = Vector2Add(boid.velocity, boid.acceleration);
            if (Vector2Length(boid.velocity) > max_velocity) {
                boid.velocity = Vector2Scale(Vector2Normalize(boid.velocity), max_velocity);
            }

            boid.position = Vector2Add(boid.position, boid.velocity);
            boid.acceleration = {0.0f, 0.0f}; // reset


            boid.position.x = fmodf(boid.position.x + screenWidth, screenWidth);
            boid.position.y = fmodf(boid.position.y + screenHeight, screenHeight);
            boid.hash_table_id = hash_table.put(boid.position, &boid);
        }


        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        if (is_debug) {
            DrawRectangle(0, 0, screenWidth, screenHeight, Fade(BLACK, 1.12f));
        } else {
            DrawRectangle(0, 0, screenWidth, screenHeight, Fade(BLACK, 0.12f));
        }


        //Draws HashTable
        if (is_debug) {
            for (int i = 0; i < hash_table.max_width_cells; ++i) {
                DrawLine(CELL_SIZE * i, 0,CELL_SIZE * i, screenHeight, LIGHTGRAY);
            }
            for (int i = 0; i < hash_table.max_height_cells; ++i) {
                DrawLine(0, CELL_SIZE * i, screenWidth, CELL_SIZE * i, LIGHTGRAY);
            }

            for (auto index: hash_table.get_indexes_of_seen_cells(boids[0].hash_table_id, scan_range_in_cells)) {
                int x = index % hash_table.max_width_cells;
                int y = index / hash_table.max_width_cells;
                DrawRectangleLines(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE, YELLOW);
            }
        }


        for (const auto &boid: boids) {
            DrawCircleV(boid.position, BOID_RADIUS, (Color){38, 104, 106, 255});
            // Vector2 direction = Vector2Scale(Vector2Normalize(boid.velocity), BOID_RADIUS * 2.0f);
            // Vector2 endpoint = Vector2Add(boid.position, direction);
            // DrawLineV(boid.position, endpoint, BLACK);
        }

        if (is_debug) {
            DrawCircleLinesV(boids[0].position, alignment_range, GREEN);
            DrawCircleLinesV(boids[0].position, separation_range, GREEN);
        }


        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
