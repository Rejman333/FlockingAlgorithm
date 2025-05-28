#include <array>
#include <iostream>
#include <string>
#include <random>

#include "raylib.h"
#include "raymath.h"
#include "data_structures/hash_table.h"

#define NUMBER_OF_BOIDS 400
#define BOID_RADIUS 2
#define CELL_SIZE 50

struct boid {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;

    int hash_table_id;
};


void fill_boids(std::array<boid, NUMBER_OF_BOIDS> &boids, const int screenWidth, const int screenHeight,
                const HashTable &hash_table) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution distribution_x(0.0f, static_cast<float>(screenWidth));
    std::uniform_real_distribution distribution_y(0.0f, static_cast<float>(screenHeight));
    std::uniform_real_distribution distribution(-5.0f, 5.0f);


    for (auto &boid: boids) {
        boid.position = {distribution_x(gen), distribution_y(gen)};
        boid.velocity = {distribution(gen), distribution(gen)};
        boid.acceleration = {.0f, 0.f};

        boid.hash_table_id = hash_table.get_cell_id(boid.position);
    }
}

inline float calculate_distance_squared(const Vector2 &from, const Vector2 &to) noexcept {
    const float dx = to.x - from.x;
    const float dy = to.y - from.y;
    return dx * dx + dy * dy;
}

Vector2 wrap_position(Vector2 pos, const float screenWidth, const float screenHeight) {
    if (pos.x < 0) pos.x = screenWidth;
    else if (pos.x > screenWidth) pos.x = 0;

    if (pos.y < 0) pos.y = screenHeight;
    else if (pos.y > screenHeight) pos.y = 0;

    return pos;
}

/*
Możesz nie normalizować pojedynczych diff wektorów,
wtedy separacja zależy tylko od różnicy pozycji, bez skalowania siły względem odległości.
Możesz nie normalizować końcowego wektora,
jeśli chcesz, by siła separacji zależała od liczby i rozkładu sąsiadów (np. tłok = silniejsze odpychanie).
*/
void separation(const int boid_id,
                std::array<boid, NUMBER_OF_BOIDS> &boids,
                const std::vector<std::pair<int, float> > &neighbors_index_distance,
                float separation_range_squared, float separation_strength) {
    Vector2 separation_force = {0.0f, 0.0f};
    int separation_count = 0;
    for (auto [n_index, dist]: neighbors_index_distance) {
        if (dist < separation_range_squared && dist > 0.0f) {
            Vector2 diff = Vector2Subtract(boids[boid_id].position, boids[n_index].position);
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
        boids[boid_id].acceleration = Vector2Add(boids[boid_id].acceleration, separation_force);
    }
}

void alignment(const int boid_id,
               std::array<boid, NUMBER_OF_BOIDS> &boids,
               const std::vector<std::pair<int, float> > &neighbors_index_distance,
               float alignment_range_squared, float alignment_strength) {
    Vector2 alignment_force = {0.0f, 0.0f};
    int alignment_count = 0;

    for (auto [n_index, dist]: neighbors_index_distance) {
        if (dist < alignment_range_squared && dist > 0.0f) {
            alignment_force = Vector2Add(alignment_force, boids[n_index].velocity);
            alignment_count++;
        }
    }

    if (alignment_count > 0) {
        alignment_force = Vector2Scale(alignment_force, 1.0f / static_cast<float>(alignment_count));
        alignment_force = Vector2Normalize(alignment_force);
        alignment_force = Vector2Scale(alignment_force, alignment_strength);
        boids[boid_id].acceleration = Vector2Add(boids[boid_id].acceleration, alignment_force);
    }
}

void cohesion(const int boid_id,
              std::array<boid, NUMBER_OF_BOIDS> &boids,
              const std::vector<std::pair<int, float> > &neighbors_index_distance,
              float cohesion_range_squared, float cohesion_strength) {
    Vector2 center_of_mass = {0.0f, 0.0f};
    int cohesion_count = 0;

    for (auto [n_index, dist]: neighbors_index_distance) {
        if (dist < cohesion_range_squared && dist > 0.0f) {
            center_of_mass = Vector2Add(center_of_mass, boids[n_index].position);
            cohesion_count++;
        }
    }

    if (cohesion_count > 0) {
        center_of_mass = Vector2Scale(center_of_mass, 1.0f / static_cast<float>(cohesion_count));
        Vector2 direction_to_center = Vector2Subtract(center_of_mass, boids[boid_id].position);
        direction_to_center = Vector2Normalize(direction_to_center);
        direction_to_center = Vector2Scale(direction_to_center, cohesion_strength);
        boids[boid_id].acceleration = Vector2Add(boids[boid_id].acceleration, direction_to_center);
    }
}



int main() {
    // Initialization
    //--------------------------------------------------------------------------------------
    constexpr int screenWidth = 1200;
    constexpr int screenHeight = 800;

    const auto hash_table = HashTable(screenWidth, screenHeight,CELL_SIZE);
    std::array<boid, NUMBER_OF_BOIDS> boids{};
    fill_boids(boids, screenWidth, screenHeight, hash_table);


    constexpr float separation_range = 20.0f;
    constexpr float alignment_range = 50.0f;
    constexpr float cohesion_range = 50.0f;

    constexpr float separation_range_squared = separation_range * separation_range;
    constexpr float alignment_range_squared = alignment_range * alignment_range;
    constexpr float cohesion_range_squared = cohesion_range * cohesion_range;

    const float separation_strength = 0.85f;
    const float alignment_strength = 0.8f;
    const float cohesion_strength = 0.8f;

    const float max_velocity = 4.0f;
    const float max_force = 0.20f;

    const float fov_angle_radians = DEG2RAD * 60.0f;
    const int scan_range_in_cells = static_cast<int>(std::max(separation_strength, alignment_strength, cohesion_strength)/CELL_SIZE);

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Boids");
    SetTargetFPS(60); // Set our game to run at 60 frames-per-second

    // Main game loop
    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        for (auto boid: boids) {
            //This is an optimization that works because fov_angle_radians <=  DEG2RAD * 90.0f
            std::vector<int> cell_index(scan_range_in_cells * scan_range_in_cells + 1);
            if(boid.velocity.x >= 0) {

            }
            else{}
            if(boid.velocity.y >= 0){}
            else{}



            for (int i = 0; i < boids.size(); ++i) { }
            std::vector<std::pair<int, float> > neighbors_index_distance;
            neighbors_index_distance.reserve(NUMBER_OF_BOIDS);


            if(boid)


            for (int j = 0; j < boids.size(); j++) {
                const float distance = calculate_distance_squared(boids[i].position, boids[j].position);
                if (distance < cohesion_range_squared) {
                    Vector2 distance_vector = Vector2Subtract(boids[j].position, boids[i].position);
                    float angle = Vector2Angle(boids[i].velocity, distance_vector);
                    if (angle < fov_angle_radians) neighbors_index_distance.emplace_back(j, distance);
                }
            }
            separation(i, boids, neighbors_index_distance, separation_range_squared, separation_strength);
            alignment(i, boids, neighbors_index_distance, alignment_range_squared, alignment_strength);
            cohesion(i, boids, neighbors_index_distance, cohesion_range_squared, cohesion_strength);
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 m_pos = GetMousePosition();
            for (auto &boid: boids) {
                const float distance = calculate_distance_squared(boid.position, m_pos);
                if (distance < 40000.f) {
                    Vector2 direction_to_center = Vector2Subtract(m_pos, boid.position);
                    boid.acceleration = Vector2Add(boid.acceleration, direction_to_center);
                }
            }
        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 m_pos = GetMousePosition();
            for (auto &boid: boids) {
                const float distance = calculate_distance_squared(boid.position, m_pos);
                if (distance < 40000.f) {
                    Vector2 direction_to_center = Vector2Subtract(m_pos, boid.position);
                    boid.acceleration = Vector2Subtract(boid.acceleration, direction_to_center);
                }
            }
        }

        // teleports to other side, and adjust velocity
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
        }


        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        DrawRectangle(0, 0, screenWidth, screenHeight, Fade(BLACK, 0.12f));

        for (const auto &boid: boids) {
            DrawCircleV(boid.position, BOID_RADIUS, (Color){38, 104, 106, 255});
            // Vector2 direction = Vector2Scale(Vector2Normalize(boid.velocity), BOID_RADIUS * 2.0f);
            // Vector2 endpoint = Vector2Add(boid.position, direction);
            // DrawLineV(boid.position, endpoint, BLACK);
        }

        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
