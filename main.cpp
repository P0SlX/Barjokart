#include "toml.hpp"
#include "astar.h"


int main() {
    std::string name, prefix = "input/";
    toml::table params;
    int choix;

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



    // Choix de la fonction d'accélération
    while (true) {
        std::cout << " 1 : Methode d'acceleration avec bresenham" << std::endl;
        std::cout << " 2 : Methode d'acceleration secondaire sans bresenham " << std::endl;
        std::cout << "Choix : ";
        std::cin >> choix;
        if (choix == 1 or choix == 2) {
            break;
        }
    }


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

    std::cout << "Lissage du chemin... ";
    std::cout.flush();
    // Lissage x5 pour être sûr d'avoir un chemin lisse et (on espère) le plus cour   /*
    for (int i = 0; i < 3; i++) {
        path = astar.lissage(path);
        ptr_paths.push_back(path);
        path = astar.lissage_naive(path);
        ptr_paths.push_back(path);
    }
    std::cout << "terminé " << "\n";


    // On grave le chemin dans l'image
    unsigned char color_mag[] = {255, 0, 255};
    for (Node *n: *path) {
        img->draw_point(n->x, n->y, color_mag);
    }

    std::vector<Pair> *speedVector;

    if (choix == 1) {
        path = astar.acceleration(path);
        ptr_paths.push_back(path);
        speedVector = AStar::nodesToSpeedVector(path);
    } else if (choix == 2) {
        auto speedVector2 = AStar::nodesToSpeedVector(path);
        auto cutvector = AStar::cutting(*speedVector2);
        speedVector = astar.acceleration2(*speedVector2, *cutvector);

        delete speedVector2;
        delete cutvector;
    }

    // Écriture du vecteur vitesse dans le fichier
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