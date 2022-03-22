#include "toml.hpp"
#include "astar.h"

int main() {
    std::string name, prefix = "input/";
    toml::table params;

    std::cout << "Nom du fichier :";
    std::cin >> name;
    std::string imagePath = prefix + name + ".png";


    // ----- Parsing du fichier TOML -----
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


    // On charge l'image
    auto img = new cimg_library::CImg<unsigned char>(imagePath.c_str());

    // On crée la matrice
    Map *map = new Map(img, colors, acc_max);

    // Pour les petites images (100x100), ont peut les affichers dans le terminal pour debug
//    map->print();

    // On crée l'objet A* et on lui passe la matrice et les coordonnées de départ
    AStar astar = AStar(start, map);

    // On lance l'algorithme A*
    auto *path = astar.aStarSearch();

    if (path == nullptr) {
        std::cout << "Aucun chemin trouvé" << std::endl;
        delete map;
        return 1;
    }

    // Nécessaire pour free les chemins précédents
    std::vector<std::vector<Node *> *> ptr_paths;
    ptr_paths.reserve(5);

    std::cout << "Lissage du chemin... ";
    std::cout.flush();
    // Lissage x5 pour être sûr d'avoir un chemin lisse et (on espère) le plus court
    for (int i = 0; i < 3; i++) {
        path = astar.lissage(path);
        ptr_paths.push_back(path);
        path = astar.lissage_naive(path);
        ptr_paths.push_back(path);
    }
    std::cout << "Terminé." << std::endl;


    // On grave le chemin dans l'image
    unsigned char color_mag[] = {255, 0, 255};
    for (Node *n: *path) {
        img->draw_point(n->x, n->y, color_mag);
    }


    path = astar.acceleration(path);

    auto *speedVector = AStar::nodesToSpeedVector(path);

    AStar::writeFile(*speedVector, "output/" + name + ".bin");


    // On écrit l'image dans un fichier .png
    std::string outputFilename = "output/" + name + ".png";
    img->save(outputFilename.c_str());

    std::cout << "Score Barjokart : " << 10000 - speedVector->size() + 1 << std::endl;

    // On libère la mémoire
    delete map;
    delete speedVector;

    for (auto ptr: ptr_paths) {
        delete ptr;
    }

    return 0;
}