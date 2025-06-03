import matplotlib.pyplot as plt
import numpy as np
import sys
from matplotlib.patches import Patch
from matplotlib.colors import ListedColormap, BoundaryNorm

file_name = sys.argv[1]

prefix = {0: "_AStar_preferred", 1: "_Dijkstra_preferred", 2: "_AStar", 3: "_Dijkstra"}
plt.figure(figsize=(12, 10))

for prefix_index in range(4):
    grid_path = "../benchmark/" + file_name + ".grid"
    route_path = "../out/" + file_name + prefix[prefix_index] + ".route"

    with open(grid_path, "r") as grid_in:
        grid_lines = grid_in.readlines()
    with open(route_path, "r") as route_in:
        route_lines = route_in.readlines()

    Net_num = int(route_lines[0])
    X_size = int(grid_lines[0].split()[0])
    Y_size = int(grid_lines[0].split()[1])
    route = [line.split() for line in route_lines[1:]]

    grid = [line.split() for line in grid_lines[1:]]
    grid = np.array(grid).astype(int)
    grid[grid >= 1] = 1

    i = 0
    while i < len(route):
        if len(route[i]) == 1:
            i += 1
            path = []
            while i < len(route) and len(route[i]) == 4:
                _, layer_str, x_str, y_str = route[i]
                layer = int(layer_str)
                x = int(x_str)
                y = int(y_str)
                path.append((layer, x, y))
                i += 1
            for j, (layer, x, y) in enumerate(path):
                idx = y + (layer - 1) * Y_size
                if j == 0 or j == len(path) - 1:
                    grid[idx][x] = 5
                else:
                    grid[idx][x] = 4
        else:
            i += 1

    cmap = ListedColormap(['gray', 'white', 'blue', 'red'])
    bounds = [-1.5, 0.5, 1.5, 4.5, 5.5]
    norm = BoundaryNorm(bounds, cmap.N)

    plt.subplot(4, 2, 1 + 2 * prefix_index)
    plt.imshow(grid[0:Y_size, :], cmap=cmap, norm=norm)
    plt.title(file_name + prefix[prefix_index] + " Layer 1")

    plt.subplot(4, 2, 2 + 2 * prefix_index)
    plt.imshow(grid[Y_size:2*Y_size, :], cmap=cmap, norm=norm)
    plt.title(file_name + prefix[prefix_index] + " Layer 2")

plt.tight_layout()

legend_elements = [
    Patch(facecolor='gray', edgecolor='k', label='Obstacle'),
    Patch(facecolor='white', edgecolor='k', label='Unused'),
    Patch(facecolor='blue', edgecolor='k', label='Routing Path'),
    Patch(facecolor='red', edgecolor='k', label='Pin/Endpoint')
]
plt.figlegend(handles=legend_elements, loc='upper center', ncol=4, framealpha=0.8, fontsize='small')

plt.savefig(f"../out/{file_name}_result.pdf")
plt.show()
