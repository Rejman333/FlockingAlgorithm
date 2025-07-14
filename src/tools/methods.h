#pragma once

enum METHOD {
    HASH,
    TREE,
    FORCE,
};

inline std::string method_to_string(METHOD method) {
    switch (method) {
        case HASH:  return "HASH";
        case TREE:  return "TREE";
        case FORCE: return "FORCE";
        default:    return "UNKNOWN";
    }
}