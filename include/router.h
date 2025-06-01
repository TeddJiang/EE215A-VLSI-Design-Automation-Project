// router.h
#ifndef ROUTER_H
#define ROUTER_H

#include <tuple>
#include <vector>
#include <fstream>
#include "parse.h"

// Type definitions
using Cost = int;
using HCost = int;
using Path = std::vector<std::tuple<int, int, int>>;
using PIN = std::tuple<Cost, int, int, int, HCost>;

// Global toggle for A* vs Dijkstra
extern bool ASTAR;

// Functor for priority queue ordering
struct Compare {
    bool operator()(const PIN &a, const PIN &b) const;
};

// Enumeration for backtracking directions
enum Direction { left = 1, right, up, down, layer_up, layer_down };

// Core routing functions
HCost ComputeHeuristic(const PIN &A, const PIN &B, const NetGrid &net_grid);
bool PINS_EQUAL(const PIN &A, const PIN &B);

// Perform routing for net at index; 'redo' indicates fallback mode
std::pair<Path, Cost> route(NetGrid net_grid, unsigned index, int redo = 0);

// Output a computed path to file
void printPath(std::ofstream &file, Path &path, NetGrid &net_grid, int net_id);

#endif // ROUTER_H

