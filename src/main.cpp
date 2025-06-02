#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <random>

#include "raylib.h"
#include "raymath.h"
#include "data_structures/hash_table.h"
#include "data_structures/Boid.h"
#include "data_structures/QuadTree.h"
#include "data_structures/k_means.h"

#include "tools/logger.h"

#define K_CLASTERS 5
#define BOID_RADIUS 2



enum METHOD {
    HASH,
    TREE,
    FORCE,
};

struct SimulationConfig {
    int width = 1200;
    int height = 800;
    int boid_count = 5000;
    bool debug_mode = false;
    METHOD method = HASH;

    float separation_range = 15.0f;
    float alignment_range = 50.0f;
    float cohesion_range = 60.0f;

    float separation_range_squared = separation_range * separation_range;
    float alignment_range_squared = alignment_range * alignment_range;
    float cohesion_range_squared = cohesion_range * cohesion_range;

    float separation_strength = 1.3f;
    float alignment_strength = 0.6f;
    float cohesion_strength = 0.8f;

    float max_velocity = 3.5f;
    float max_force = 0.10f;

    float fov_angle_radians = 60.0f * DEG2RAD;

    int cell_size = 50;
    int max_boids_in_tree = 10;

    int max_neighbors = 20;
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
        } else if (arg == "-cell_size" && i + 1 < argc) {
            config.cell_size = std::stoi(argv[++i]);
        } else if (arg == "-max_tree" && i + 1 < argc) {
            config.max_boids_in_tree = std::stoi(argv[++i]);
        } else if (arg == "-max_neighbors" && i + 1 < argc) {
            config.max_neighbors = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
        }
    }
}

Vector2 wrap_position(Vector2 pos, const float screenWidth, const float screenHeight) {
    if (pos.x < 0) pos.x = screenWidth;
    else if (pos.x > screenWidth) pos.x = 0;

    if (pos.y < 0) pos.y = screenHeight;
    else if (pos.y > screenHeight) pos.y = 0;

    return pos;
}



LogConfig log_cfg{.method_name = "HashTable", .number_of_boids = NUMBER_OF_BOIDS};
logger Logger(log_cfg);


int main(int argc, char *argv[]) {
    // Initialization
    //--------------------------------------------------------------------------------------
    SimulationConfig config;
    parse_args(argc, argv, config);


    SetConfigFlags(FLAG_MSAA_4X_HINT);
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
    switch (config.method) {
        case HASH:
            hash_table.build(boids);
            break;
        case TREE:
            quad_tree.build(boids);
            break;
        case FORCE:
            return 1;
            break;

        default:
            hash_table.build(boids);
    }


    std::vector<int> claster_assingment_to_blolid;
    std::vector<Color> claster_colors = generate_random_colors(K_CLASTERS);

    auto last_kmeans_time = std::chrono::high_resolution_clock::now();
    // Main game loop
    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        Logger.startRetrievalTimer();
        std::vector<Boid *> boids_in_range;


        for (auto &boid: boids) {
            boids_in_range.clear();
            neighbors.clear();

            switch (config.method) {
                case HASH:
                    boids_in_range = hash_table.get_boids_in_range(boid.hash_table_id);
                    break;
                case TREE:
                    boids_in_range = quad_tree.query(boid.position, config.cohesion_range);
                    break;
                case FORCE:
                    return 1;
                    break;

                default:
                    boids_in_range = hash_table.get_boids_in_range(boid.hash_table_id);
            }

            std::ranges::shuffle(boids_in_range, rng);

            for (auto boid_in_range: boids_in_range) {
                if (boid_in_range == &boid) continue;

                float dist_sqr = Vector2DistanceSqr(boid.position, boid_in_range->position);
                if (dist_sqr > config.cohesion_range_squared) continue;

                Vector2 diff = Vector2Subtract(boid_in_range->position, boid.position);
                float angle = Vector2Angle(boid.velocity, diff);
                if (angle > config.fov_angle_radians) continue;

                neighbors.push_back({boid_in_range, dist_sqr});
                if (neighbors.size() > config.max_neighbors - 1) break;
            }

            apply_boid_behaviors(boid, neighbors,
                                 config.separation_range_squared, config.separation_strength,
                                 config.alignment_range_squared, config.alignment_strength,
                                 config.cohesion_range_squared, config.cohesion_strength);
        }

        Logger.stopRetrievalTimer();

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

        Logger.startBuildTimer();
        hash_table.reset();
        quad_tree.reset();


        for (auto &boid: boids) {
            if (Vector2Length(boid.acceleration) > config.max_force) {
                boid.acceleration = Vector2Scale(Vector2Normalize(boid.acceleration), config.max_force);
            }

            boid.velocity = Vector2Add(boid.velocity, boid.acceleration);
            if (Vector2Length(boid.velocity) > config.max_velocity) {
                boid.velocity = Vector2Scale(Vector2Normalize(boid.velocity), config.max_velocity);
            }

            boid.position = Vector2Add(boid.position, boid.velocity);
            boid.acceleration = {0.0f, 0.0f}; // reset


            boid.position.x = fmodf(boid.position.x + static_cast<float>(config.width),
                                    static_cast<float>(config.width));
            boid.position.y = fmodf(boid.position.y + static_cast<float>(config.height),
                                    static_cast<float>(config.height));


            //rebuild datastructures
            if (config.method == TREE) {
                quad_tree.insert(&boid);
            } else if (config.method == HASH) {
                boid.hash_table_id = hash_table.put(boid.position, &boid);
            }
        }

        Logger.stopBuildTimer();


        static int frame_counter = 0;
        if (frame_counter++ % 300 == 0) { // np. 600 klatek przy 60 FPS = 10 sekund
            run_kmeans(boid_ptrs, K_CLASTERS, claster_assingment_to_blolid);
        }



        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
        if (config.debug_mode) {
            DrawRectangle(0, 0, config.width, config.height, Fade(BLACK, 1.12f));
        } else {
            DrawRectangle(0, 0, config.width, config.width, Fade(BLACK, 0.12f));
        }

        for (const auto &boid: boids) {
            DrawCircleV(boid.position, BOID_RADIUS, (Color){38, 104, 106, 255});
        }


        //Draws HashTable
        if (config.debug_mode && config.method == HASH) {
            for (int i = 0; i < hash_table.max_width_cells; ++i) {
                DrawLine(config.cell_size * i, 0, config.cell_size * i, config.height, LIGHTGRAY);
            }
            for (int i = 0; i < hash_table.max_height_cells; ++i) {
                DrawLine(0, config.cell_size * i, config.width, config.cell_size * i, LIGHTGRAY);
            }
            for (auto index: hash_table.get_indexes_of_seen_cells(boids[0].hash_table_id)) {
                int x = index % hash_table.max_width_cells;
                int y = index / hash_table.max_width_cells;
                DrawRectangleLines(x * config.cell_size, y * config.cell_size, config.cell_size, config.cell_size, YELLOW);
            }
        }


        for (size_t j = 0; j < boids.size(); ++j) {
            const Boid& boid = boids[j];
            int cluster_id = claster_assingment_to_blolid[j];
            Color color = claster_colors[cluster_id];
            DrawCircleV(boid.position, BOID_RADIUS, color);
        }
        //for (const auto &boid: boids) {
          //  DrawCircleV(boid.position, BOID_RADIUS, (Color){38, 104, 106, 255});
            // Vector2 direction = Vector2Scale(Vector2Normalize(boid.velocity), BOID_RADIUS * 2.0f);
            // Vector2 endpoint = Vector2Add(boid.position, direction);
            // DrawLineV(boid.position, endpoint, BLACK);
        //}

        if (config.debug_mode && config.method == TREE) {
            quad_tree.draw(5);
            quad_tree.draw_t(boids[0].position, config.cohesion_range, 3);
        }

        if (config.debug_mode) {
            DrawCircleV(boids[0].position, BOID_RADIUS, RED);
            DrawCircleLinesV(boids[0].position, config.alignment_range, GREEN);
            DrawCircleLinesV(boids[0].position, config.separation_range, GREEN);
        }

        DrawFPS(10, 10);

        // logger save to file
        Logger.updateInfoFPS(static_cast<float>(GetFPS()));
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double>(now - Logger.last_log_time).count();
        if (duration >= 10.0) {
            Logger.saveToFile();
        }


        EndDrawing();
    }

    CloseWindow();
    return 0;
}
