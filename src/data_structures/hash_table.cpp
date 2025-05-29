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


std::vector<std::vector<Boid *> > HashTable::get_boids_from_cells_in_range(
    const int cell_index, const int scan_range) const {
    std::vector<std::vector<Boid *> > to_return;
    int centrum_x_index = cell_index % max_width_cells;
    int centrum_y_index = cell_index / max_width_cells;

    const int start_x = std::max(centrum_x_index - scan_range, 0);
    const int end_x = std::min(centrum_x_index + scan_range, max_width_cells);

    const int start_y = std::max(centrum_y_index - scan_range, 0);
    const int end_y = std::min(centrum_y_index + scan_range, max_height_cells);

    for (int i = start_x; i <= end_x; ++i) {
        for (int j = start_y; j < end_y; ++j) {
            to_return.push_back(cells[i + j * max_width_cells]);
        }
    }

    return to_return;
}

std::vector<Boid *> &HashTable::get_boids_at_index(const int cell_index) {
    return cells[cell_index];
}

std::vector<int> HashTable::get_indexes_of_seen_cells(const int cell_index, const int scan_range) const {
        std::vector<int> to_return;
        int centrum_x_index = cell_index % max_width_cells;
        int centrum_y_index = cell_index / max_width_cells;

        const int start_x = std::max(centrum_x_index - scan_range, 0);
        const int end_x = std::min(centrum_x_index + scan_range, max_width_cells);

        const int start_y = std::max(centrum_y_index - scan_range, 0);
        const int end_y = std::min(centrum_y_index + scan_range, max_height_cells);

        for (int i = start_x; i <= end_x; ++i) {
            for (int j = start_y; j <= end_y; ++j) {
                to_return.push_back(i + j * max_width_cells);
            }
        }

        return to_return;
}
