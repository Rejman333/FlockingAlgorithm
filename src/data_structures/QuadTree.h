#pragma once
#include <array>

#include "Boid.h"

template<int MAX_CAPACITY>
class QuadTree {
    const Rectangle boundary{0, 0, 0, 0};
    std::array<Boid *, MAX_CAPACITY> elements;
    int number_of_elements = 0;
    bool divided = false;

    QuadTree *northeast = nullptr;
    QuadTree *northwest = nullptr;
    QuadTree *southeast = nullptr;
    QuadTree *southwest = nullptr;

public:
    QuadTree() = default;

    explicit QuadTree(const Rectangle boundary): boundary(boundary) {
    };

    void build(std::vector<Boid> &boids) {
        for (auto &boid: boids) {
            insert(&boid);
        }
    }

    ~QuadTree() {
        if (northeast) {
            delete northeast;
            northeast = nullptr;
        }
        if (northwest) {
            delete northwest;
            northwest = nullptr;
        }
        if (southeast) {
            delete southeast;
            southeast = nullptr;
        }
        if (southwest) {
            delete southwest;
            southwest = nullptr;
        }
    }

    void reset() {
        if (northeast) {
            delete northeast;
            northeast = nullptr;
        }
        if (northwest) {
            delete northwest;
            northwest = nullptr;
        }
        if (southeast) {
            delete southeast;
            southeast = nullptr;
        }
        if (southwest) {
            delete southwest;
            southwest = nullptr;
        }
        divided = false;
    }

    void insert(Boid *p_boid) {
        if (!divided) {
            if (number_of_elements >= MAX_CAPACITY) {
                subdivide();
            } else {
                elements[number_of_elements++] = p_boid;
                return;
            }
        }
        if (CheckCollisionPointRec(p_boid->position, northeast->boundary)) {
            northeast->insert(p_boid);
            return;
        }
        if (CheckCollisionPointRec(p_boid->position, northwest->boundary)) {
            northwest->insert(p_boid);
            return;
        }
        if (CheckCollisionPointRec(p_boid->position, southeast->boundary)) {
            southeast->insert(p_boid);
            return;
        }
        if (CheckCollisionPointRec(p_boid->position, southwest->boundary)) {
            southwest->insert(p_boid);
            return;
        }
    }

    void subdivide() {
        divided = true;

        float halfWidth = boundary.width / 2.0f;
        float halfHeight = boundary.height / 2.0f;
        float x = boundary.x;
        float y = boundary.y;

        // NW: lewa góra
        Rectangle nwRect = {x, y, halfWidth, halfHeight};
        northwest = new QuadTree(nwRect);

        // NE: prawa góra
        Rectangle neRect = {x + halfWidth, y, halfWidth, halfHeight};
        northeast = new QuadTree(neRect);

        // SW: lewa dół
        Rectangle swRect = {x, y + halfHeight, halfWidth, halfHeight};
        southwest = new QuadTree(swRect);

        // SE: prawa dół
        Rectangle seRect = {x + halfWidth, y + halfHeight, halfWidth, halfHeight};
        southeast = new QuadTree(seRect);


        for (int i = 0; i < number_of_elements; ++i) {
            Boid *boid = elements[i];
            if (CheckCollisionPointRec(boid->position, northeast->boundary)) {
                northeast->insert(boid);
            } else if (CheckCollisionPointRec(boid->position, northwest->boundary)) {
                northwest->insert(boid);
            } else if (CheckCollisionPointRec(boid->position, southeast->boundary)) {
                southeast->insert(boid);
            } else if (CheckCollisionPointRec(boid->position, southwest->boundary)) {
                southwest->insert(boid);
            }
        }

        number_of_elements = 0;
    }

    std::vector<Boid *> query(const Vector2 &center, const float radius) {
        std::vector<Boid *> neighbors;
        if (divided) {
            if (CheckCollisionCircleRec(center, radius, northeast->boundary)) {
                auto ne = northeast->query(center, radius);
                neighbors.insert(neighbors.end(), ne.begin(), ne.end());
            }
            if (CheckCollisionCircleRec(center, radius, northwest->boundary)) {
                auto nw = northwest->query(center, radius);
                neighbors.insert(neighbors.end(), nw.begin(), nw.end());
            }
            if (CheckCollisionCircleRec(center, radius, southeast->boundary)) {
                auto se = southeast->query(center, radius);
                neighbors.insert(neighbors.end(), se.begin(), se.end());
            }
            if (CheckCollisionCircleRec(center, radius, southwest->boundary)) {
                auto sw = southwest->query(center, radius);
                neighbors.insert(neighbors.end(), sw.begin(), sw.end());
            }
        } else {
            for (int i = 0; i < number_of_elements; ++i) {
                float dx = elements[i]->position.x - center.x;
                float dy = elements[i]->position.y - center.y;
                float distSq = dx * dx + dy * dy;
                if (distSq <= radius * radius) {
                    neighbors.push_back(elements[i]);
                }
            }
        }
        return neighbors;
    }

    void draw(float line_thickness) const {
        DrawRectangleLinesEx(boundary, line_thickness, DARKGRAY);
        float new_thickness = std::max(line_thickness - 1, 1.f);
        if (northeast) {
            northeast->draw(new_thickness);
        }
        if (northwest) {
            northwest->draw(new_thickness);
        }
        if (southeast) {
            southeast->draw(new_thickness);
        }
        if (southwest) {
            southwest->draw(new_thickness);
        }
    }

    void draw_t(const Vector2 &center, const float radius, float line_thickness) const {
        float new_thickness = std::max(line_thickness - 1, 1.f);
        if (divided) {
            if (CheckCollisionCircleRec(center, radius, northeast->boundary)) {
                northeast->draw_t(center, radius, new_thickness);
            }
            if (CheckCollisionCircleRec(center, radius, northwest->boundary)) {
                northwest->draw_t(center, radius, new_thickness);
            }
            if (CheckCollisionCircleRec(center, radius, southeast->boundary)) {
                southeast->draw_t(center, radius, new_thickness);
            }
            if (CheckCollisionCircleRec(center, radius, southwest->boundary)) {
                southwest->draw_t(center, radius, new_thickness);
            }
        } else {
            DrawRectangleLinesEx(boundary, line_thickness, YELLOW);
        }
    }
};
