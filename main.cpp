#include <vector>
#include <iostream>
#include "astar.h"

int main() {

    // Fake grid
    Node *grid[10][10];
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            grid[i][j] = new Node(i, j, 0, 0, 0, nullptr);
        }
    }
    Node *startNode = grid[3][6];
    Node *endNode = grid[9][9];


    std::vector<Node *> oui = a_star(grid, startNode, endNode);

    print_grid_path(grid, oui);

    // free grid
    for (auto & i : grid) {
        for (auto & j : i) {
            delete j;
        }
    }

    return 0;
}
