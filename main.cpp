#include "astar.h"

int main() {
    // TODO
    //  - TOML support for src and dest
    //  - Use of pointers for A*

    // start coord
   Pair src = Pair(0, 0);

    // destination color
    int dest[3] = {255, 0, 0};

    std::string path = "hills.png";

    AStar astar(src, dest, path);
    astar.aStarSearch();

   /* std::vector<Pair> v1;
    Pair src1 = Pair(2, 2);
    v1.push_back(src);
    v1.push_back(src1);
    astar.writeFile(v1);*/



    return 0;
}
