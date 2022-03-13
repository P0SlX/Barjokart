#include "toml.hpp"
#include "astar.h"

int main() {
    std::string name;
    toml::table params;

    std::cout << "Nom du fichier :";
    std::cin >> name;

    try {
        params = toml::parse_file("../input/" + name + ".toml");
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    int acc_max = params["acc_max"].value<int>().value();
    Pair start = {params["depart"]["x"].value<int>().value(), params["depart"]["y"].value<int>().value()};
    int colors[3] = {params["couleur_arrivee"][0].value<int>().value(),
                     params["couleur_arrivee"][1].value<int>().value(),
                     params["couleur_arrivee"][2].value<int>().value()};

    std::cout << "Acceleration max : " << acc_max << std::endl;
    std::cout << "Coordonnees de depart : {" << start.first << ", " << start.second << "}" << std::endl;
    std::cout << "Couleur d'arrivee : {" << colors[0] << ", " << colors[1] << ", " << colors[2] << "}" << std::endl;

    AStar aStar(start, colors, name);
    AStar::writeFile(AStar::speedVector(aStar.aStarSearch()), name);

    return 0;
}