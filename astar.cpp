#include "astar.h"

AStar::AStar(Pair &src, int *dest, std::string &filename) : src(std::move(src)), filename(std::move(filename)) {
    this->img = new cimg_library::CImg<unsigned char>(this->filename.c_str());

    this->height = img->height();
    this->width = img->width();


    // instantiate the grid with vector of vector of node pointers
    this->grid = new std::vector<std::vector<Node *>>;
    for (int i = 0; i < this->height; i++) {
        this->grid->emplace_back();
        for (int j = 0; j < this->width; j++) {
            (*this->grid)[i].push_back(new Node());
        }
    }

    // Transforming the image into a grid of 0s and 1s
    // And at the same time, storing the coordinates of the destinations points
    for (int i = 0; i < this->height; i++) {
        for (int j = 0; j < this->width; j++) {
            bool isFree = (*this->img)(j, i, 0) == 255 && (*this->img)(j, i, 1) == 255 && (*this->img)(j, i, 2) == 255;
            bool isGreen = (*this->img)(j, i, 0) == 0 && (*this->img)(j, i, 1) == 255 && (*this->img)(j, i, 2) == 4;
            bool isDest = (*this->img)(j, i, 0) == dest[0] && (*this->img)(j, i, 1) == dest[1] &&
                          (*this->img)(j, i, 2) == dest[2];

            // Chemin libre ou point de départ
            if (isFree || isGreen)
                (*this->grid)[j][i]->is_wall = false;

            else if (isDest) {   // Destination
                this->dest.emplace_back(j, i);
                (*this->grid)[j][i]->is_wall = false;

            } else
                (*this->grid)[j][i]->is_wall = true;
        }
    }

    // Display grid with finish points
    std::cout << "Grille avec le(s) point(s) d'arrivée(s) : " << std::endl;
    for (int k = 0; k < this->height; k++) {
        for (int l = 0; l < this->width; l++) {
            // Ce if est super lent donc pour la version finale il faudra commenter
            // C'est juste à des fins de débug pour savoir sont les points d'arrivées
            if (find(this->dest.begin(), this->dest.end(), Pair(l, k)) != this->dest.end()) {
                std::cout << "X ";
                continue;
            }

            if ((*this->grid)[l][k]->is_wall)
                std::cout << "██";
            else
                std::cout << "  ";
        }
        // \n pour pas flush le buffer et gagner du temps
        std::cout << "\n";
    }
}


AStar::~AStar() {
    for (int i = 0; i < this->height; i++) {
        for (int j = 0; j < this->width; j++) {
            delete (*this->grid)[i][j];
        }
    }
    delete this->img;
}


bool AStar::isValid(const Pair &point) const {
    // Check si le point est dans la grille
    if (this->height > 0 && this->width > 0)
        return (point.second >= 0) && (point.second < this->height) && (point.first >= 0) &&
               (point.first < this->width);
    return false;
}


bool AStar::isUnBlocked(const Pair &point) const {
    // Si le point est dans la grille et n'est pas un mur
    return isValid(point) && !(*this->grid)[point.first][point.second]->is_wall;
}


double AStar::heuristic(const Pair &source) const {
    int min_heuristic = INT_MAX;
    int heuristic;
    for (Pair d: this->dest) {
        heuristic = abs(source.first - d.first) + abs(source.second - d.second);
        if (heuristic < min_heuristic)
            min_heuristic = heuristic;
    }
    return min_heuristic;
}


void AStar::tracePath(Pair &d) {
    std::vector<Pair> path;

    int i = d.first, j = d.second;
    Pair next_node = (*this->grid)[j][i]->parent;
    do {
        path.push_back(next_node);
        next_node = (*this->grid)[j][i]->parent;
        i = next_node.first;
        j = next_node.second;
    } while ((*this->grid)[j][i]->parent != next_node);

    path.emplace_back(i, j);
    path.push_back(d);

    // save path on image
    for (Pair p: path) {
        const unsigned char color_mag[] = {0, 255, 0};
        this->img->draw_point(p.first, p.second, color_mag);
    }
    this->img->save("output.png");
}


void AStar::aStarSearch() {
    if (!isValid(this->src)) {
        printf("Le point source n'est pas dans l'image\n");
        return;
    }

    for (Pair d: this->dest)
        if (!isValid(d)) {
            printf("Le(s) point(s) de destination n'est/sont pas dans l'image\n");
            return;
        }

    if (!isUnBlocked(this->src)) {
        printf("Le point source est un obstacle\n");
        return;
    }

    for (Pair d: this->dest)
        if (!isUnBlocked(d)) {
            printf("Le(s) point(s) de destination est/sont un obstacle\n");
            return;
        }

    for (Pair d: this->dest)
        if (this->src == d) {
            printf("Le point source est déjà dans la destination\n");
            return;
        }

    bool closedList[this->height][this->width];
    memset(closedList, false, sizeof(closedList));

    int i, j;
    i = src.first, j = src.second;
    (*this->grid)[j][i]->f = 0.0;
    (*this->grid)[j][i]->g = 0.0;
    (*this->grid)[j][i]->h = 0.0;
    (*this->grid)[j][i]->parent = {i, j};

    std::priority_queue<Tuple, std::vector<Tuple>, std::greater<> > openList;

    openList.emplace(0.0, j, i);

    while (!openList.empty()) {
        const Tuple &p = openList.top();
        j = get<1>(p);
        i = get<2>(p);

        openList.pop();
        closedList[j][i] = true;

        for (int add_x = -1; add_x <= 1; add_x++) {
            for (int add_y = -1; add_y <= 1; add_y++) {
                Pair neighbour(i + add_x, j + add_y);
                if (isValid(neighbour)) {
                    for (Pair d: this->dest) {
                        if (d.first == neighbour.first && d.second == neighbour.second) {
                            (*this->grid)[neighbour.second][neighbour.first]->parent = {i, j};
                            printf("Le point de destination à été atteint\n");
                            this->tracePath(d);
                            return;
                        } else if (!closedList[neighbour.second][neighbour.first] && isUnBlocked(neighbour)) {
                            double gNew = (*this->grid)[j][i]->g + 1.0;
                            double hNew = heuristic(neighbour);
                            double fNew = gNew + hNew;

                            if ((*this->grid)[neighbour.second][neighbour.first]->f == -1 ||
                                    (*this->grid)[neighbour.second][neighbour.first]->f > fNew) {
                                openList.emplace(fNew, neighbour.second, neighbour.first);

                                (*this->grid)[neighbour.second][neighbour.first]->g = gNew;
                                (*this->grid)[neighbour.second][neighbour.first]->h = hNew;
                                (*this->grid)[neighbour.second][neighbour.first]->f = fNew;
                                (*this->grid)[neighbour.second][neighbour.first]->parent = {i, j};
                            }
                        }
                    }
                }
            }
        }
    }
    printf("Impossible de trouver le chemin\n");


}


void AStar::writeFile(std::vector<Pair> &vecteur) {
    std::fstream fichier;
    std::string nomfichier = "equipe4.bin";
    fichier.open(nomfichier, std::ios::out | std::ios::binary);
    if (!fichier.is_open()) {
        std::cout << "Impossible d'ecrire le fichier est deja ouvert " << nomfichier << '\n';
    } else {
        std::vector<Pair>::iterator it;
        for (it = vecteur.begin(); it != vecteur.end(); it++) {
            int x = it->first;
            int y = it->second;
            fichier.write((char *) &x, sizeof(int));
            fichier.write((char *) &y, sizeof(int));
        }
        printf("Ecriture reussie\n");
        fichier.close();
    }
}
