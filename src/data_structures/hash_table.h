#pragma once
#include <stdexcept>

#include "raylib.h"


class HashTable {
    int cell_size;
    int width_in_cells;
    int height_in_cells;
    int number_of_buckets;

public:
    HashTable(): cell_size(0), width_in_cells(0), height_in_cells(0), number_of_buckets(0) {
    };

    HashTable(int space_width, int space_height, int cell_size): cell_size(cell_size) {
        if (space_width <= 0 || space_height <= 0 || cell_size <= 0)
            throw std::invalid_argument(
                "Width, height and cell size must be positive integers.");
        height_in_cells = space_height / cell_size;
        width_in_cells = space_width / cell_size;
        number_of_buckets = height_in_cells * width_in_cells;
    }

    [[nodiscard]] int get_cell_id(Vector2 position) const;
};
