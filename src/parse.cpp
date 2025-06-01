// Parsing the netlist file and grid file
#include "parse.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string.h>


// 字符串分割为整数数组
std::vector<int> split(const std::string &str) {
    std::vector<int> tokens;
    std::stringstream ss(str);
    std::string token;
    while (ss >> token)
        tokens.push_back(std::stoi(token));
    return tokens;
}

// 按曼哈顿距离排序网
bool compareDistance(const Net &a, const Net &b) {
    return a.distance < b.distance;
}

// 解析输入文件
NetGrid parsefile(std::string file_name) {
    std::string grid_path = "../benchmark/" + file_name + ".grid";
    std::string nl_path   = "../benchmark/" + file_name + ".nl";

    std::ifstream grid_in(grid_path);
    std::ifstream nl_in(nl_path);
    std::vector<std::string> grid_lines, nl_lines;
    std::string line;

    while (getline(grid_in, line)) grid_lines.push_back(line);
    while (getline(nl_in, line)) nl_lines.push_back(line);

    NetGrid net_grid;
    std::vector<int> params = split(grid_lines[0]);
    net_grid.X_size = params[0];
    net_grid.Y_size = params[1];
    net_grid.Bend_Penalty = params[2];
    net_grid.Via_Penalty  = params[3];
    net_grid.Net_num = std::stoi(nl_lines[0]);

    std::vector<std::vector<int>> grid_data;
    for (size_t i = 1; i < grid_lines.size(); ++i)
        grid_data.push_back(split(grid_lines[i]));

    Grid grid(net_grid.X_size, std::vector<std::vector<int>>(net_grid.Y_size, std::vector<int>(2, 0)));
    for (int i = 0; i < net_grid.Y_size; ++i) {
        for (int j = 0; j < net_grid.X_size; ++j) {
            grid[j][i][0] = grid_data[i][j];
            grid[j][i][1] = grid_data[net_grid.Y_size + i][j];
        }
    }

    std::vector<Net> nets(net_grid.Net_num);
    for (int i = 1; i <= net_grid.Net_num; ++i) {
        std::vector<int> nd = split(nl_lines[i]);
        for (int j = 0; j < 7; ++j)
            nets[i - 1].data[j] = nd[j];

        nets[i - 1].distance = std::abs(nd[2] - nd[5]) + std::abs(nd[3] - nd[6]);
    }

    std::vector<Net> x_nets, y_nets, l_nets;
    for (const auto &net : nets) {
        if (net.data[2] == net.data[5])
            y_nets.push_back(net);
        else if (net.data[3] == net.data[6])
            x_nets.push_back(net);
        else
            l_nets.push_back(net);
    }

    std::sort(x_nets.begin(), x_nets.end(), compareDistance);
    std::sort(y_nets.begin(), y_nets.end(), compareDistance);
    std::sort(l_nets.begin(), l_nets.end(), compareDistance);

    nets.clear();
    nets.insert(nets.end(), x_nets.begin(), x_nets.end());
    nets.insert(nets.end(), y_nets.begin(), y_nets.end());
    nets.insert(nets.end(), l_nets.begin(), l_nets.end());

    net_grid.nets = nets;
    net_grid.grid = grid;
    return net_grid;
}

// 打印网格与网络信息
void printNetGrid(const NetGrid &net_grid, const std::string &output_file) {
    std::ofstream out(output_file);

    out << "Nets:\n";
    for (const auto &net : net_grid.nets) {
        out << "Net: ";
        for (int i = 0; i < 7; ++i)
            out << net.data[i] << " ";
        out << "Distance: " << net.distance << "\n";
    }

    out << "Grid Layer 0:\n";
    for (int y = 0; y < net_grid.Y_size; ++y) {
        for (int x = 0; x < net_grid.X_size; ++x)
            out << net_grid.grid[x][y][0] << " ";
        out << "\n";
    }

    out << "Grid Layer 1:\n";
    for (int y = 0; y < net_grid.Y_size; ++y) {
        for (int x = 0; x < net_grid.X_size; ++x)
            out << net_grid.grid[x][y][1] << " ";
        out << "\n";
    }

    out.close();
}