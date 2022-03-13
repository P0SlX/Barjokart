#include "toml.hpp"
#include "astar.h"

int main() {
    std::string name, prefix = "input/";
    toml::table params;

    std::cout << "Nom du fichier :";
    std::cin >> name;
    std::string imagePath = prefix + name + ".png";

    try {
        params = toml::parse_file(prefix + name + ".toml");
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    int acc_max = params["acc_max"].value<int>().value();
    Pair start = {params["depart"]["x"].value<int>().value(), params["depart"]["y"].value<int>().value()};

    unsigned char colors[3] = {
            params["couleur_arrivee"][0].value<unsigned char>().value(),
            params["couleur_arrivee"][1].value<unsigned char>().value(),
            params["couleur_arrivee"][2].value<unsigned char>().value()
    };

    std::cout << "Acceleration max : " << acc_max << std::endl;
    std::cout << "Coordonnees de depart : (" << start.first << ", " << start.second << ")" << std::endl;
    std::cout << "Couleur d'arrivee : [" << (int) colors[0] << ", " << (int) colors[1] << ", " << (int) colors[2] << "]"
              << std::endl;


    auto img = new cimg_library::CImg<unsigned char>(imagePath.c_str());

    Map *map = new Map(img, colors);
//    map->print();

    AStar astar = AStar(start, map);
    auto *path = astar.aStarSearch();
    auto *speedVector = astar.nodesToSpeedVector(path);

    std::string outputFilename = "output/" + name + ".bin";
    AStar::writeFile(*speedVector, outputFilename);

    // write path to image (debug)
    for (auto &p: *path) {
        const unsigned char color_mag[] = {0, 255, 0};
        img->draw_point(p->x, p->y, color_mag);
    }
    img->save("output/debug.png");

    delete map;
    delete speedVector;
    return 0;
}