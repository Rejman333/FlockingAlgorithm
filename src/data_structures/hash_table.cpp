#include "hash_table.h"

int HashTable::get_cell_id(const Vector2 &position) const {
    return (static_cast<int>(position.x) / cell_size) + (static_cast<int>(position.y) / cell_size) * max_width_cells;
}

int HashTable::put(const Vector2 &position, Boid *p_boid) {
    const int cell_id = get_cell_id(position);
    cells[cell_id].push_back(p_boid);
    return cell_id;
};

//This cant be const
void HashTable::reset() {
    for (auto &cell: cells) {
        cell.clear(); // czyści oryginalny wektor wewnętrzny
    }
}


std::vector<Boid*> HashTable::get_boids_in_range(int cell_index) const {
    std::vector<Boid*> result;

    int cx = cell_index % max_width_cells;
    int cy = cell_index / max_width_cells;

    const int start_y = std::max(cy - scan_range, 0);
    const int end_y = std::min(cy + scan_range, max_height_cells - 1);
    const int start_x = std::max(cx - scan_range, 0);
    const int end_x = std::min(cx + scan_range, max_width_cells - 1);

    // opcjonalnie: prealokuj na oko
    result.reserve((end_x - start_x + 1) * (end_y - start_y + 1) * 5);
    for (int y = start_y; y <= end_y; ++y) {
        for (int x = start_x; x <= end_x; ++x) {
            int index = x + y * max_width_cells;
            const auto& cell = cells[index];
            result.insert(result.end(), cell.begin(), cell.end());
        }
    }

    return result;
}

std::vector<Boid *> &HashTable::get_boids_at_index(const int cell_index) {
    return cells[cell_index];
}

std::vector<int> HashTable::get_indexes_of_seen_cells(const int cell_index) const {
    std::vector<int> result;
    result.reserve((2 * scan_range + 1) * (2 * scan_range + 1));

    const int cx = cell_index % max_width_cells;
    const int cy = cell_index / max_width_cells;

    const int start_y = std::max(cy - scan_range, 0);
    const int end_y = std::min(cy + scan_range, max_height_cells - 1);
    const int start_x = std::max(cx - scan_range, 0);
    const int end_x = std::min(cx + scan_range, max_width_cells - 1);

    for (int y = start_y; y <= end_y; ++y) {
        const int row_offset = y * max_width_cells;
        for (int x = start_x; x <= end_x; ++x) {
            result.push_back(row_offset + x);
        }
    }

    return result;
}
