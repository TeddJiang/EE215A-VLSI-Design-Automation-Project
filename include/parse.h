#ifndef PARSE_H
#define PARSE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string.h>

struct Net {
    int data[7];
    int distance;
};

using Grid = std::vector<std::vector<std::vector<int>>>;

struct NetGrid {
    std::vector<Net> nets;
    Grid grid;
    int Via_Penalty;
    int Bend_Penalty;
    int X_size;
    int Y_size;
    int Net_num;
};

std::vector<int> split(const std::string &str);
bool compareDistance(const Net &a, const Net &b);
NetGrid parsefile(std::string file_name);
void printNetGrid(const NetGrid &net_grid, const std::string &output_file);

#endif
