#include "astar.h"

int main() {
    // TODO
    //  - TOML support for src and dest
    //  - Use of pointers for A*

    // start coord
    Pair src = Pair(0, 0);

    // destination color
    int dest[3] = {255, 0, 0};

    std::string path = "adaptallure.png";

    AStar astar(src, dest, path);
    astar.aStarSearch();
    return 0;
}
