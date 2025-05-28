#pragma once
#include <stdexcept>
#include <vector>

#include "raylib.h"


class HashTable {
    int cell_size;
    int width_in_cells;
    int height_in_cells;
    int number_of_buckets;
    std::vector<std::vector<int> > cells;

public:
    HashTable(): cell_size(0), width_in_cells(0), height_in_cells(0), number_of_buckets(0),
                 cells(std::vector<std::vector<int> >(0)) {
    };

    HashTable(int space_width, int space_height, int cell_size): cell_size(cell_size) {
        if (space_width <= 0 || space_height <= 0 || cell_size <= 0)
            throw std::invalid_argument(
                "Width, height and cell size must be positive integers.");
        height_in_cells = space_height / cell_size;
        width_in_cells = space_width / cell_size;
        number_of_buckets = height_in_cells * width_in_cells;
        cells.reserve(number_of_buckets);
        for (int i = 0; i < number_of_buckets; ++i) {
            cells.emplace_back(10);
        }
    }

    void put(const Vector2& position, int i);

    void reset() const;

    [[nodiscard]] int get_cell_id(const Vector2 &position) const;
};
