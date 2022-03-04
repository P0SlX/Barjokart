#include <vector>
#include "astar.h"

int main() {

    int HEIGHT = 10;
    int WIDTH = 10;

    // Fake grid
    std::vector<std::vector<Node*>> grid_vector;
    for (int i = 0; i < HEIGHT; i++) {
        std::vector<Node *> row;
        row.reserve(WIDTH);
        for (int j = 0; j < WIDTH; j++) {
            row.emplace_back(new Node(i, j,0,0,0, nullptr));
        }
        grid_vector.emplace_back(row);
    }
    Node *startNode = grid_vector[3][6];
    Node *endNode = grid_vector[9][9];


    std::vector<Node *> oui = a_star(grid_vector, startNode, endNode);

    print_grid_path(grid_vector, oui);

    // free grid
    for (auto & i : grid_vector) {
        for (auto & j : i) {
            delete j;
        }
    }

    return 0;
}
