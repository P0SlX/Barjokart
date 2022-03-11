#include "astar.h"
#include "toml.hpp"

int main() {
    // TODO
    //  - TOML support for src and dest
    //  - Use of pointers for A*

    /*Pair src = Pair(0, 0);
    int dest[3] = {255, 0, 0};
    toml::table tbl;
    try {
        tbl = toml::parse_file("../circuit1.toml");
        int acc_max = tbl["acc_max"].value<int>().value();
        std::cout << "Acceleration " << acc_max << "\n";
        auto couleur = tbl["couleur_arrivee"];
        int r = couleur[0].value<int>().value();
        int g = couleur[1].value<int>().value();
        int b = couleur[2].value<int>().value();
        std::cout << "couleur " << r + " " << g + " " + b << "\n";
        auto depart = tbl["depart"];
        int x = depart["x"].value<int>().value();
        int y = depart["y"].value<int>().value();
        std::cout << x + " " + y << "\n";
    }
    catch (const toml::parse_error &e) {
        printf("Erreur de chargement\n");
    }*/

    Pair src = Pair(89, 92);
    int dest[3] = {255, 0, 0};
    std::string path = "hills.png";
    AStar astar(src, dest, path);
    AStar::writeFile(AStar::speedVector(astar.aStarSearch()));

    return 0;
}
