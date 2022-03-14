#include "map.h"

Map::Map(cimg_library::CImg<unsigned char> *img, const unsigned char *color_dest) {
    this->img = img;

    std::cout << "Création de la matrice en cours... ";
    std::cout.flush();

    // Initialize the map
    for (int i = 0; i < img->width(); i++) {
        for (int j = 0; j < img->height(); j++) {
            Node *node = new Node(i, j);
            map.push_back(node);

            // Si c'est un pixel de destination
            if (img->atXY(i, j, 0) == (int) color_dest[0] && img->atXY(i, j, 1) == (int) color_dest[1] &&
                img->atXY(i, j, 2) == (int) color_dest[2]) {
                node->isDestination = true;
                this->destinationsNodes.push_back(node);
            }

            // Si c'est un mur
            if (img->atXY(i, j, 0) == 0 && img->atXY(i, j, 1) == 0 && img->atXY(i, j, 2) == 0) {
                node->isWall = true;
            }
        }
    }

    // Set the neighbors of each node
    for (int i = 0; i < img->width(); i++) {
        for (int j = 0; j < img->height(); j++) {
            Node *node = getNode(i, j);

            // Gauche
            if (i > 0) {
                node->neighbors->push_back(std::make_pair(getNode(i - 1, j), 1));
                // Haut gauche
                if (j > 0)
                    node->neighbors->push_back(std::make_pair(getNode(i - 1, j - 1), 1));
            }

            // Droite
            if (i < img->width() - 1) {
                node->neighbors->push_back(std::make_pair(getNode(i + 1, j), 1));
                // Haut droite
                if (j > 0)
                    node->neighbors->push_back(std::make_pair(getNode(i + 1, j - 1), 1));
            }

            // Haut
            if (j > 0)
                node->neighbors->push_back(std::make_pair(getNode(i, j - 1), 1));

            // Bas
            if (j < img->height() - 1) {
                node->neighbors->push_back(std::make_pair(getNode(i, j + 1), 1));
                // Bas gauche
                if (i > 0)
                    node->neighbors->push_back(std::make_pair(getNode(i - 1, j + 1), 1));
            }

            // Diagonale bas droite
            if (i < img->width() - 1 && j < img->height() - 1)
                node->neighbors->push_back(std::make_pair(getNode(i + 1, j + 1), 1));
        }
    }

    // Comme on a plusieurs destinations, on pourrait prendre la plus proche
    // Sauf que s'il y a 17k points... Ca prend beaucoup de temps
    // Donc on fait la moyenne des coordonnées des destinations
    // Pour avoir à la fin un point central
    double x = 0;
    double y = 0;
    for (auto &i: this->destinationsNodes) {
        x += i->x;
        y += i->y;
    }
    x /= (double) this->destinationsNodes.size();
    y /= (double) this->destinationsNodes.size();
    this->destination = getNode((int) x, (int) y);

    std::cout << "Terminé." << std::endl;
}


Map::~Map() {
    for (auto &i: map) {
        delete i;
        i = nullptr;
    }
    delete this->img;
    this->img = nullptr;
}

void Map::print() {
    for (int i = 0; i < img->width(); i++) {
        for (int j = 0; j < img->height(); j++) {
            Node *node = getNode(i, j);

            if (node->isWall)
                std::cout << "██";
            else if (this->destination == node)
                std::cout << "F ";
            else if (node->isDestination)
                std::cout << "D ";
            else
                std::cout << "  ";
        }
        std::cout << std::endl;
    }
}
