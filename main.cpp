#include "astar.h"

int main() {
    AStar astar(Pair(89, 92), Pair(88, 50), "hills.png");
    astar.aStarSearch();
    return 0;
}
