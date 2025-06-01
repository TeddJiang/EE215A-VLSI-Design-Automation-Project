// router.cpp

#include "router.h"
#include <queue>
#include <limits>
#include <cmath>
#include <iostream>

// Global toggle for A* vs Dijkstra
bool ASTAR = false;

// Compare functor implementation
bool Compare::operator()(const PIN &a, const PIN &b) const {
    // Each PIN is: (g, layer, x, y, f = g + h)
    return std::get<4>(a) > std::get<4>(b);
}

// Heuristic: Manhattan distance + via penalty if layers differ
inline HCost ComputeHeuristic(const PIN &A, const PIN &B, const NetGrid &net_grid) {
    int x1 = std::get<2>(A), y1 = std::get<3>(A), l1 = std::get<1>(A);
    int x2 = std::get<2>(B), y2 = std::get<3>(B), l2 = std::get<1>(B);
    HCost d = std::abs(x1 - x2) + std::abs(y1 - y2);
    if (l1 != l2) d += net_grid.Via_Penalty;
    return d;
}

// Check if two PINs have the same layer/x/y (ignore g and f)
bool PINS_EQUAL(const PIN &A, const PIN &B) {
    return std::get<1>(A) == std::get<1>(B)
        && std::get<2>(A) == std::get<2>(B)
        && std::get<3>(A) == std::get<3>(B);
}

std::pair<Path, Cost> route(NetGrid net_grid, unsigned index, int redo) {
    auto &nt = net_grid.nets[index];
    // Extract start and end PINs
    PIN start = {0, nt.data[1], nt.data[2], nt.data[3], 0};
    PIN end   = {0, nt.data[4], nt.data[5], nt.data[6], 0};

    // If endpoint is blocked (grid value ¡Ü 0), abort immediately
    if (net_grid.grid[std::get<2>(end)]
                     [std::get<3>(end)]
                     [std::get<1>(end) - 1] <= 0)
    {
        // Do not print a message here per request; just return empty path
        return {{}, 0};
    }

    // Initialize f for start: f = g (0) + h if A* is enabled
    if (ASTAR) {
        std::get<4>(start) = ComputeHeuristic(start, end, net_grid);
    } else {
        std::get<4>(start) = 0;
    }

    // Open set: priority queue ordered by f = g + h
    std::priority_queue<PIN, std::vector<PIN>, Compare> open;
    open.push(start);

    int X = net_grid.X_size;
    int Y = net_grid.Y_size;

    // Closed set: track visited (layer 1 or 2) at each (x, y)
    std::vector<std::vector<std::vector<bool>>> closed(
        X, std::vector<std::vector<bool>>(Y, std::vector<bool>(2, false)));

    // Predecessor grid for backtracking: stores a Direction (1..6) at each (x, y, layer-1)
    std::vector<std::vector<std::vector<int>>> pred(
        X, std::vector<std::vector<int>>(Y, std::vector<int>(2, 0)));

    // Direction indexing:
    // left=1, right=2, up=3, down=4, layer_up=5, layer_down=6
    // offset[d] = {¦¤layer, ¦¤x, ¦¤y}
    int offset[7][3] = {
        { 0,  0,  0},   // dummy index 0
        { 0,  0, -1},   // left
        { 0,  0, +1},   // right
        { 0, -1,  0},   // up
        { 0, +1,  0},   // down
        {+1,  0,  0},   // layer_up
        {-1,  0,  0}    // layer_down
    };

    int steps = 0;
    const int MAX_STEPS = X * Y * 20;
    PIN curr;

    while (!open.empty()) {
        curr = open.top();
        open.pop();

        int l = std::get<1>(curr);
        int x = std::get<2>(curr);
        int y = std::get<3>(curr);
        Cost g = std::get<0>(curr);

        // If already closed, skip
        if (closed[x][y][l - 1]) continue;
        closed[x][y][l - 1] = true;

        // If we've reached the end PIN, break
        if (PINS_EQUAL(curr, end)) {
            std::cerr << "[route] Net " << index
                      << " reached end in " << steps
                      << " steps, cost=" << g << "\n";
            std::cerr.flush();
            break;
        }

        // If too many expansions, abort
        if (++steps > MAX_STEPS) {
            std::cerr << "[route] Net " << index
                      << " reached MAX_STEPS, aborting\n";
            std::cerr.flush();
            break;
        }

        // Lambda to expand neighbors
        auto expand = [&](int nl, int nx, int ny, int dir) {
            // Bounds check
            if (nx < 0 || nx >= X || ny < 0 || ny >= Y) return;
            // If already closed or grid cell is blocked (¡Ü 0), skip
            if (closed[nx][ny][nl - 1] || net_grid.grid[nx][ny][nl - 1] <= 0) return;

            // New g = current g + grid cost at (nx, ny, nl-1)
            Cost ng = g + net_grid.grid[nx][ny][nl - 1];

            // Determine if bending penalty applies:
            // if direction is horizontal (left/right) and previous direction was vertical (up/down), or vice versa
            int pd = pred[x][y][l - 1];
            bool horizontal = (dir == left || dir == right);
            bool vertical   = (dir == up   || dir == down);
            bool prev_horiz = (pd == left  || pd == right);
            bool prev_vert  = (pd == up    || pd == down);
            if ((horizontal && prev_vert) || (vertical && prev_horiz)) {
                ng += net_grid.Bend_Penalty;
            }

            // Via penalty if layer changes
            if (nl != l) {
                ng += net_grid.Via_Penalty;
            }

            // Compute f = g + h if A* is enabled, otherwise f = g
            HCost nf = ASTAR
                ? ng + ComputeHeuristic({ng, nl, nx, ny, 0}, end, net_grid)
                : ng;

            open.emplace(ng, nl, nx, ny, nf);
            pred[nx][ny][nl - 1] = dir;
        };

        // Expand possible moves:
        // 1) Via: switch layer
        expand((l == 1 ? 2 : 1), x, y, (l == 1 ? layer_up : layer_down));
        // 2) Left / Right
        expand(l, x, y - 1, left);
        expand(l, x, y + 1, right);
        // 3) Up / Down (only if redo == 1)
        if (redo) {
            expand(l, x - 1, y, up);
            expand(l, x + 1, y, down);
        }
    }

    // If open set emptied without reaching end, return empty
    if (!PINS_EQUAL(curr, end)) {
        return {{}, 0};
    }

    // Otherwise, backtrack to build Path
    Path path;
    Cost total = std::get<0>(curr);

    // Backtrack until we reach start
    while (!PINS_EQUAL(curr, start)) {
        int cl = std::get<1>(curr);
        int cx = std::get<2>(curr);
        int cy = std::get<3>(curr);
        path.insert(path.begin(), {cl, cx, cy});

        int d = pred[cx][cy][cl - 1];
        cx -= offset[d][1];
        cy -= offset[d][2];
        cl -= offset[d][0];
        curr = {0, cl, cx, cy, 0};
    }

    // Finally insert the start itself
    path.insert(path.begin(),
                { std::get<1>(start),
                  std::get<2>(start),
                  std::get<3>(start) });

    return {path, total};
}

// Print the computed path to file
void printPath(std::ofstream &file, Path &path, NetGrid &net_grid, int net_id) {
    for (auto &pt : path) {
        int l, x, y;
        std::tie(l, x, y) = pt;
        file << net_id << " " << l << " " << x << " " << y << "\n";
    }
}
