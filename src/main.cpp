#include <iostream>
#include <string>
#include <chrono>
#include "clipp.h"
#include "parse.h"
#include "router.h"
using namespace clipp;
using namespace std;

int main(int argc, char *argv[])
{
    std::string filename = "bench1";
    size_t strategy = 0;
    size_t a_star = 0;
    auto cli = (
        option("-f", "--filename") & value("filename", filename).doc("The input filename"),
        option("-s", "--strategy") & value("strategy 0/1", strategy)
            .doc("Set the strategy to do the routing, 0 focus on only x or y, 1 focus on both"),
        option("-a", "--astar") & value("a_star 0/1", a_star)
            .doc("Whether to use the A* algorithm")
    );

    if (!parse(argc, argv, cli))
    {
        cout << make_man_page(cli, argv[0]);
        return 0;
    }
    ASTAR = (a_star != 0);
    std::cout << "The input file name is: " << filename << std::endl;

    auto net_grid = parsefile(filename);

    auto start = std::chrono::high_resolution_clock::now();

    unsigned correct = 0;
    unsigned total_cost = 0;
    std::string output_file_name = "../out/" + filename;

    if (a_star == 1)
        output_file_name += "_AStar";
    else
        output_file_name += "_Dijkstra";
    if (strategy == 0)
        output_file_name += "_preferred";
    output_file_name += ".route";

    ofstream output_file(output_file_name);
    output_file << net_grid.Net_num << "\n";

    for (int i = 0; i < net_grid.Net_num; i++)
    {
        //std::cerr << "-> main: about to call route for net " << i << "\n";
        auto result = route(net_grid, i, strategy);
        //std::cerr << "<- main: returned from route for net " << i << "\n";

        total_cost += result.second;
        output_file << net_grid.nets[i].data[0] << "\n";

        if (result.first.empty())
        {
            //std::cerr << "-> main: route failed, retrying with redo=1 for net " << i << "\n";
            result = route(net_grid, i, 1);
            //std::cerr << "<- main: returned from redo route for net " << i << "\n";

            total_cost += result.second;
            //std::cerr << "Current net is " << i << "\n";

            if (result.first.empty())
                output_file << "0\n";
            else
            {
                printPath(output_file, result.first, net_grid, net_grid.nets[i].data[0]);
                correct++;
            }
        }
        else
        {
            printPath(output_file, result.first, net_grid, net_grid.nets[i].data[0]);
            correct++;
        }
        //std::cerr << "\n";
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;

    std::cout << "Routed PINS: " << correct << "/" << net_grid.Net_num << std::endl;
    std::cout << "Total Cost: " << total_cost << std::endl;
    std::cout << "Execution time: " << duration.count() << " ms" << std::endl;

    return 0;
}
