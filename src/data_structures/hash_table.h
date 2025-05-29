#pragma once
#include <stdexcept>
#include <vector>

#include "raylib.h"
#include "Boid.h"


class HashTable {
    std::vector<std::vector<Boid*> > cells;

public:
    int cell_size;
    int max_width_cells;
    int max_height_cells;
    int number_of_buckets;


    HashTable(): cells(std::vector<std::vector<Boid*> >(0)), cell_size(0), max_width_cells(0), max_height_cells(0),
                 number_of_buckets(0) {
    };

    HashTable(const int space_width, const int space_height, const int cell_size): cell_size(cell_size) {
        if (space_width <= 0 || space_height <= 0 || cell_size <= 0)
            throw std::invalid_argument(
                "Width, height and cell size must be positive integers.");

        max_height_cells = space_height / cell_size;
        max_width_cells = space_width / cell_size;
        number_of_buckets = std::max(max_height_cells * max_width_cells, 1);

        for (int i = 0; i < number_of_buckets; ++i) {
            cells.emplace_back();
        }
    }

    int put(const Vector2 &position, Boid *p_boid);

    void reset();

    [[nodiscard]] std::vector<std::vector<Boid*>> get_boids_from_cells_in_range(int cell_index, int scan_range) const;

    std::vector<Boid *>& get_boids_at_index(int cell_index);

    std::vector<int> get_indexes_of_seen_cells(int cell_index, int scan_range) const;

    [[nodiscard]] int get_cell_id(const Vector2 &position) const;

};
