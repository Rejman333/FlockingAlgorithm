#include "hash_table.h"

int HashTable::get_cell_id(const Vector2 &position) const {
    return (static_cast<int>(position.x) / cell_size) + (static_cast<int>(position.y) / cell_size) * width_in_cells;
}

void HashTable::put(const Vector2 &position, const int i) {
    cells[get_cell_id(position)].push_back(i);
};

void HashTable::reset() const {
    for (auto cell: cells) {
        cell.clear();
    }
}
