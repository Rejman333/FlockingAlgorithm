#pragma once

#include "raylib.h"

struct Boid {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;

    int hash_table_id;
};