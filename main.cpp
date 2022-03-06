#include "astar.h"

int main() {
    // TODO
    //  - TOML support for src and dest

    // start coord
    Pair src = Pair(89, 92);

    // destination color
    int dest[3] = {255, 0, 0};

    std::string path = "hills.png";

    AStar astar(src, dest, path);
    astar.aStarSearch();
    return 0;
}
