#include "map.h"
#include "astar.h"

int main() {
    auto img = new cimg_library::CImg<unsigned char>("hills.png");
    unsigned char destination_color[3] = {255, 0, 0};

    Pair start_point(89, 92);

    Map *map = new Map(img, destination_color);
//    map->print();

    AStar astar = AStar(start_point, map);
    auto *path = astar.aStarSearch();
    auto *speedVector = astar.nodesToSpeedVector(path);
    AStar::writeFile(*speedVector);

    // write path to image (debug)
    for (auto &p : *path) {
        const unsigned char color_mag[] = {0, 255, 0};
        img->draw_point(p->x, p->y, color_mag);
    }
    img->save("output.png");


    delete map;
    delete speedVector;
    return 0;
}
