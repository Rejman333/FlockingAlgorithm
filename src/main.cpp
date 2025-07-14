#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <random>
#include <ranges>

#include "raylib.h"
#include "raymath.h"
#include "data_structures/hash_table.h"
#include "data_structures/Boid.h"
#include "data_structures/QuadTree.h"
#include "data_structures/k_means.h"
#include "tools/MyLogger.h"
#include "tools/methods.h"


#define BOID_RADIUS 2
Color transparentGray = {LIGHTGRAY.r, LIGHTGRAY.g, LIGHTGRAY.b, 80};
Color transparent_yellow = {YELLOW.r, YELLOW.g, YELLOW.b, 80};


struct SimulationConfig {
    int width = 1200;
    int height = 800;
    int boid_count = 12000;
    bool debug_mode = false;
    METHOD method = HASH;

    bool k_mean = false;
    int k_mean_clusters = 3;
    int k_mean_max_iter = 3;

    float separation_range = 10.0f;
    float alignment_range = 60.0f;
    float cohesion_range = 60.0f;

    float separation_range_squared = separation_range * separation_range;
    float alignment_range_squared = alignment_range * alignment_range;
    float cohesion_range_squared = cohesion_range * cohesion_range;

    float separation_strength = 4.0f;
    float alignment_strength = 1.5f;
    float cohesion_strength = 1.5f;

    float max_velocity = 2.0f;
    float max_force = 0.10f;

    float fov_angle_radians = 120.0f * DEG2RAD;
    float cos_half_fov = cosf(fov_angle_radians / 2.0f);

    int cell_size = 25;
    int max_boids_in_tree = 30;

    int max_neighbors = 10;
};


METHOD parse_method(const std::string &str) {
    if (str == "hash") return HASH;
    if (str == "qtree") return TREE;
    if (str == "brute") return FORCE;
    throw std::runtime_error("Unknown method: " + str);
}

void parse_args(int argc, char *argv[], SimulationConfig &config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-width" && i + 1 < argc) {
            config.width = std::stoi(argv[++i]);
        } else if (arg == "-height" && i + 1 < argc) {
            config.height = std::stoi(argv[++i]);
        } else if (arg == "-boids" && i + 1 < argc) {
            config.boid_count = std::stoi(argv[++i]);
        } else if (arg == "-method" && i + 1 < argc) {
            config.method = parse_method(argv[++i]);
        } else if (arg == "-debug") {
            config.debug_mode = true;
        } else if (arg == "-k_mean") {
            config.k_mean = true;
        } else if (arg == "-sep_range" && i + 1 < argc) {
            config.separation_range = std::stof(argv[++i]);
            config.separation_range_squared = config.separation_range * config.separation_range;
        } else if (arg == "-ali_range" && i + 1 < argc) {
            config.alignment_range = std::stof(argv[++i]);
            config.alignment_range_squared = config.alignment_range * config.alignment_range;
        } else if (arg == "-coh_range" && i + 1 < argc) {
            config.cohesion_range = std::stof(argv[++i]);
            config.cohesion_range_squared = config.cohesion_range * config.cohesion_range;
        } else if (arg == "-sep_str" && i + 1 < argc) {
            config.separation_strength = std::stof(argv[++i]);
        } else if (arg == "-ali_str" && i + 1 < argc) {
            config.alignment_strength = std::stof(argv[++i]);
        } else if (arg == "-coh_str" && i + 1 < argc) {
            config.cohesion_strength = std::stof(argv[++i]);
        } else if (arg == "-max_vel" && i + 1 < argc) {
            config.max_velocity = std::stof(argv[++i]);
        } else if (arg == "-max_force" && i + 1 < argc) {
            config.max_force = std::stof(argv[++i]);
        } else if (arg == "-fov" && i + 1 < argc) {
            config.fov_angle_radians = std::stof(argv[++i]) * DEG2RAD;
            config.cos_half_fov = cosf(config.fov_angle_radians / 2.0f);
        } else if (arg == "-cell_size" && i + 1 < argc) {
            config.cell_size = std::stoi(argv[++i]);
        } else if (arg == "-max_tree" && i + 1 < argc) {
            config.max_boids_in_tree = std::stoi(argv[++i]);
        } else if (arg == "-max_neighbors" && i + 1 < argc) {
            config.max_neighbors = std::stoi(argv[++i]);
        } else if (arg == "-k_mean_clusters" && i + 1 < argc) {
            config.k_mean_clusters = std::stoi(argv[++i]);
        } else if (arg == "-k_mean_max_iter" && i + 1 < argc) {
            config.k_mean_max_iter = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
        }
    }
}

int main(int argc, char *argv[]) {
    // Initialization
    //--------------------------------------------------------------------------------------
    SimulationConfig config;
    parse_args(argc, argv, config);

    //SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_ALWAYS_RUN);
    InitWindow(config.width, config.height, "Boids");
    SetTargetFPS(60);

    std::random_device rd;
    std::mt19937 rng(rd());

    //TODO set it to max of ranges
    const int scan_range = std::max(static_cast<int>(config.alignment_range) / config.cell_size, 1);
    auto hash_table = HashTable(config.width, config.height, config.cell_size, scan_range);
    auto quad_tree = QuadTree<10>(Rectangle({
        0, 0,
        static_cast<float>(config.width),
        static_cast<float>(config.height)
    }));

    std::vector<Boid> boids = fill_boids(config.boid_count, config.width, config.height);
    std::vector<std::pair<Boid *, float> > neighbors(config.boid_count);
    std::vector<Boid *> first_boid_neighbors;

    std::string filename = "../" + method_to_string(config.method) + "_" + std::to_string(config.boid_count) + ".csv";
    auto loger = MyLogger(config.boid_count, config.method, filename);

    std::vector<Color> colors = generate_random_colors(config.k_mean_clusters);
    int frame_count = 0;

    switch (config.method) {
        case HASH: {
            auto start = std::chrono::high_resolution_clock::now();
            hash_table.build(boids);
            auto end = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration<double, std::micro>(end - start).count();
            loger.recordBuildTime(duration);
            break;
        }
        case TREE: {
            auto start = std::chrono::high_resolution_clock::now();
            quad_tree.build(boids);
            auto end = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration<double, std::micro>(end - start).count();
            loger.recordBuildTime(duration);
            break;
        }
        case FORCE:
            // Brak struktury do budowy
            break;
        default: {
            auto start = std::chrono::high_resolution_clock::now();
            hash_table.build(boids);
            auto end = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration<double, std::micro>(end - start).count();
            loger.recordBuildTime(duration);
            break;
        }
    }

    std::vector<Boid *> boids_in_range;

    while (!WindowShouldClose()) {
        first_boid_neighbors.clear();
        for (auto &boid: boids) {
            boids_in_range.clear();
            neighbors.clear();

            switch (config.method) {
                case HASH: {
                    auto start = std::chrono::high_resolution_clock::now();
                    boids_in_range = hash_table.get_boids_in_range(boid.hash_table_id);
                    auto end = std::chrono::high_resolution_clock::now();
                    double duration = std::chrono::duration<double, std::micro>(end - start).count();
                    loger.recordRetrievalTime(duration);
                    break;
                }
                case TREE: {
                    auto start = std::chrono::high_resolution_clock::now();
                    boids_in_range = quad_tree.query(boid.position, config.cohesion_range);
                    auto end = std::chrono::high_resolution_clock::now();
                    double duration = std::chrono::duration<double, std::micro>(end - start).count();
                    loger.recordRetrievalTime(duration);
                    break;
                }
                case FORCE: {
                    auto start = std::chrono::high_resolution_clock::now();
                    for (auto &bf_boid: boids) {
                        boids_in_range.push_back(&bf_boid);
                    }
                    auto end = std::chrono::high_resolution_clock::now();
                    double duration = std::chrono::duration<double, std::micro>(end - start).count();
                    loger.recordRetrievalTime(duration);
                    break;
                }

                default:
                    return 1;
            }

            if (boids_in_range.size() > config.max_neighbors) {
                std::ranges::shuffle(boids_in_range, rng);
            }

            auto start = std::chrono::high_resolution_clock::now();
            for (auto boid_in_range: boids_in_range) {
                if (boid_in_range == &boid) continue;

                float dist_sqr = Vector2DistanceSqr(boid.position, boid_in_range->position);
                if (dist_sqr > config.cohesion_range_squared) continue;

                Vector2 diff = Vector2Subtract(boid_in_range->position, boid.position);
                diff = Vector2Normalize(diff);

                if (Vector2DotProduct(boid.velocity_norm, diff) < config.cos_half_fov) continue;

                neighbors.emplace_back(boid_in_range, dist_sqr);
                if (neighbors.size() > config.max_neighbors - 1) break;
            }
            if (config.debug_mode && &boid == &boids[0]) {
                for (auto key: neighbors | std::views::keys) {
                    first_boid_neighbors.push_back(key);
                }
            }

            auto end = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration<double, std::micro>(end - start).count();
            loger.recordCheckTime(duration);

            apply_boid_behaviors(boid, neighbors,
                                 config.separation_range_squared, config.separation_strength,
                                 config.alignment_range_squared, config.alignment_strength,
                                 config.cohesion_range_squared, config.cohesion_strength);
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

        //Clear Data Structures
        hash_table.reset();
        quad_tree.reset();


        for (auto &boid: boids) {
            float acc_len = Vector2Length(boid.acceleration);
            if (acc_len > config.max_force) {
                boid.acceleration = Vector2Scale(boid.acceleration, config.max_force / acc_len);
            }

            Vector2 full_velocity = Vector2Scale(boid.velocity_norm, boid.speed);

            full_velocity = Vector2Add(full_velocity, boid.acceleration);

            float new_speed = Vector2Length(full_velocity);
            if (new_speed > config.max_velocity) {
                new_speed = config.max_velocity;
                full_velocity = Vector2Scale(Vector2Normalize(full_velocity), new_speed);
            }

            boid.speed = new_speed;
            boid.velocity_norm = (new_speed > 0.001f)
                                     ? Vector2Scale(full_velocity, 1.0f / new_speed)
                                     : Vector2Zero();

            Vector2 velocity_now = Vector2Scale(boid.velocity_norm, boid.speed);
            boid.position = Vector2Add(boid.position, velocity_now);

            boid.acceleration = {0.0f, 0.0f};

            boid.position.x = fmodf(boid.position.x + static_cast<float>(config.width),
                                    static_cast<float>(config.width));
            boid.position.y = fmodf(boid.position.y + static_cast<float>(config.height),
                                    static_cast<float>(config.height));
        }

        if (config.method == TREE) {
            auto start = std::chrono::high_resolution_clock::now();
            for (auto &boid: boids) {
                quad_tree.insert(&boid);
            }
            auto end = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration<double, std::micro>(end - start).count();
            loger.recordBuildTime(duration);
        } else if (config.method == HASH) {
            auto start = std::chrono::high_resolution_clock::now();
            for (auto &boid: boids) {
                boid.hash_table_id = hash_table.put(boid.position, &boid);
            }
            auto end = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration<double, std::micro>(end - start).count();
            loger.recordBuildTime(duration);
        }

        if (config.k_mean && frame_count == 60*3) {
            switch (config.method) {
                case FORCE: k_means(boids, config.k_mean_clusters, config.k_mean_max_iter);
                    break;
                case HASH: k_means(boids, config.k_mean_clusters,config.k_mean_max_iter, &hash_table);
                    break;
                case TREE: k_means(boids, config.k_mean_clusters,config.k_mean_max_iter, &quad_tree);
                    break;
                default:
                    return 1;
            }
            frame_count = 0;
        }


        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
        if (config.debug_mode) {
            DrawRectangle(0, 0, config.width, config.height, Fade(BLACK, 1.12f));
        } else {
            DrawRectangle(0, 0, config.width, config.width, Fade(BLACK, 0.12f));
        }

        //Draws HashTable
        if (config.debug_mode && config.method == HASH) {
            for (int i = 0; i < hash_table.max_width_cells; ++i) {
                DrawLine(config.cell_size * i, 0, config.cell_size * i, config.height, transparentGray);
            }
            for (int i = 0; i < hash_table.max_height_cells; ++i) {
                DrawLine(0, config.cell_size * i, config.width, config.cell_size * i, transparentGray);
            }
            for (auto index: hash_table.get_indexes_of_seen_cells(boids[0].hash_table_id)) {
                int x = index % hash_table.max_width_cells;
                int y = index / hash_table.max_width_cells;
                DrawRectangleLines(x * config.cell_size, y * config.cell_size, config.cell_size, config.cell_size,
                                   transparent_yellow);
            }
        }
        if (config.debug_mode && config.method == TREE) {
            quad_tree.draw(3);
            quad_tree.draw_t(boids[0].position, config.cohesion_range, 3);
        }

        for (const auto &boid: boids) {
            DrawCircleV(boid.position, BOID_RADIUS, colors[boid.cluster_id]);
        }

        if (config.debug_mode) {
            for (auto boid: first_boid_neighbors) {
                DrawCircleV(boid->position, BOID_RADIUS, (Color){138, 204, 106, 255});
            }

            DrawCircleV(boids[0].position, BOID_RADIUS, RED);

            Vector2 full_velocity = Vector2Scale(boids[0].velocity_norm, boids[0].speed * 10.0f);
            Vector2 end_point = Vector2Add(boids[0].position, full_velocity);
            DrawLineV(boids[0].position, end_point, BLUE);

            DrawCircleLinesV(boids[0].position, config.alignment_range, GREEN);
            DrawCircleLinesV(boids[0].position, config.separation_range, GREEN);
        }

        DrawFPS(10, 10);
        loger.tick();
        EndDrawing();
        frame_count++;
    }

    CloseWindow();
    return 0;
}
