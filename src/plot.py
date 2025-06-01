import matplotlib.pyplot as plt
import numpy as np
import sys

file_name = sys.argv[1]

prefix = {0:"_AStar_preferred", 1:"_Dijkstra_preferred", 2:"_AStar", 3:"_Dijkstra"}
    # Show the grid after routing
plt.figure(figsize=(12, 10))
for prefix_index in range(4):
    # Data path
    grid_path = "../benchmark/" + file_name + ".grid"
    route_path = "../out/" + file_name + prefix[prefix_index] + ".route"

    # Read data
    with open(grid_path, "r") as grid_in:
        grid_lines = grid_in.readlines()
    with open(route_path, "r") as route_in:
        route_lines = route_in.readlines()

    # Basic parameters
    Net_num = int(route_lines[0])
    X_size = int(grid_lines[0].split()[0])
    Y_size = int(grid_lines[0].split()[1])
    route = [line.split() for line in route_lines[1:]]

    grid = [line.split() for line in grid_lines[1:]]
    grid = np.array(grid).astype(int)

    # Draw the route
    grid[grid >= 1] = 1

    i = 0
    while i < len(route):
        if int(route[i][0]) != 0 and len(route[i]) == 1:
            if int(route[i + 1][0]) == 2:
                layer = 1
            elif int(route[i + 1][0]) == 0:
                i += 1
                continue
            else:
                layer = 0
            grid[int(route[i + 1][2]) + layer * Y_size][int(route[i + 1][1])] = 5

            cntr = 2
            while True:
                if int(route[i + cntr][0]) == 0:
                    if int(route[i + cntr - 1][0]) == 2:
                        layer = 1
                    else:
                        layer = 0
                    grid[int(route[i + cntr - 1][2]) + layer * Y_size][int(route[i + cntr - 1][1])] = 5
                    break

                if int(route[i + cntr][0]) == 2:
                    layer = 1
                else:
                    layer = 0
                grid[int(route[i + cntr][2]) + layer * Y_size][int(route[i + cntr][1])] = 4

                cntr += 1
            i += cntr
        i += 1

    # Create a color map
    from matplotlib.colors import ListedColormap, BoundaryNorm

    # Custom color map: -1=black, 0=white, 1=gray, 4=blue, 5=red
    cmap = ListedColormap(['gray', 'black', 'white', 'blue', 'red'])
    bounds = [-1.5, -0.5, 0.5, 1.5, 4.5, 5.5]
    norm = BoundaryNorm(bounds, cmap.N)

    plt.subplot(4, 2, 1+2*prefix_index)
    plt.imshow(grid[:Y_size], cmap=cmap, norm=norm)
    plt.title(file_name+prefix[prefix_index] + " Layer 1")


    # # Overlay grid
    # for i in range(Y_size):
    #     for j in range(X_size):
    #         if grid[i][j] == 1:
    #             grid[i][j] = grid[i + Y_size][j]

    # Show the overlay grid
    plt.subplot(4, 2, 2+2*prefix_index)
    plt.imshow(grid[Y_size:], cmap=cmap, norm=norm)
    plt.title(file_name+prefix[prefix_index] + " Layer 2")


plt.tight_layout()
plt.savefig(f"../out/{file_name}_result.pdf")
plt.show()
