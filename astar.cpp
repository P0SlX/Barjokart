#include "astar.h"

AStar::AStar(Pair start, Map *map) {
    this->startNode = map->getNode(start.first, start.second);
    this->startNode->f = this->startNode->g = this->startNode->h = 0;
    this->path = new std::vector<Node *>();
    this->map = map;
}

AStar::~AStar() {
    delete this->path;
    this->path = nullptr;
}

double AStar::heuristic(Node *node, Node *dest) {
    double x_square = (node->x - dest->x) * (node->x - dest->x);
    double y_square = (node->y - dest->y) * (node->y - dest->y);
    return sqrt(x_square + y_square);
}

void AStar::reconstructPath(Node *endNode) {
    Node *parent = endNode->parent;
    this->path->push_back(endNode);
    while (parent != nullptr) {
        path->push_back(parent);
        parent = parent->parent;
    }
}

std::vector<Node *> *AStar::aStarSearch() {
    std::cout << "A* en cours... ";
    std::cout.flush();
    auto start = std::chrono::high_resolution_clock::now();


    Node *currentNode = this->startNode;
    Node *childNode;
    double f, g, h;

    std::make_heap(this->openList.begin(), this->openList.end(), compareNodes());
    pushOpen(currentNode);

    while (!this->openList.empty()) {
        std::sort(this->openList.begin(), this->openList.end(), compareNodes());

        currentNode = this->openList.front(); // pop n node from open for which f is minimal
        popOpen(currentNode);

        currentNode->isClosed = true;
        this->closedList.push_back(currentNode);

        if (currentNode->isDestination) {
            auto stop = std::chrono::high_resolution_clock::now();
            double duration = (double) duration_cast<std::chrono::microseconds>(stop - start).count();
            duration /= 1000;
            reconstructPath(currentNode);
            std::cout << "Terminé en " << duration << "ms." << std::endl;
            return this->path;
        }

        // Cette boucle est très longue si il y a beaucoup de noeuds de destination
        double h_min = std::numeric_limits<double>::max();
        for (Node *n: this->map->destinationsNodes) {
            double tmp = heuristic(currentNode, n);
            if (tmp < h_min) {
                h_min = tmp;
                this->map->destination = n;
            }
        }

        for (const auto &children: *currentNode->neighbors) {
            childNode = children.first;

            if (childNode->isWall)
                continue;

            g = currentNode->g + children.second;

            if ((childNode->isOpen || childNode->isClosed) && childNode->g <= g)
                continue;

            h = this->heuristic(childNode, this->map->destination);
            f = g + h;
            childNode->f = f;
            childNode->g = g;
            childNode->h = h;
            childNode->setParent(currentNode);

            if (!childNode->isOpen)
                pushOpen(childNode);
        }
    }
    return nullptr;
}

std::vector<Pair> *AStar::nodesToSpeedVector(std::vector<Node *> *path) {
    auto *speedVector = new std::vector<Pair>();
    std::reverse(path->begin(), path->end());
    for (int k = 0; k < path->size() - 1; k++) {
        speedVector->emplace_back((*path)[k + 1]->x - (*path)[k]->x, (*path)[k + 1]->y - (*path)[k]->y);
    }
    return speedVector;
}

void AStar::writeFile(std::vector<Pair> &vector, const std::string &filename) {
    std::ofstream file;
    file.open(filename, std::ios::binary);
    if (file.is_open()) {
        for (Pair p: vector) {
            file.write((char *) &p.first, sizeof(int));
            file.write((char *) &p.second, sizeof(int));
        }
        file.close();
        std::cout << "Écriture du fichier binaire réussie" << std::endl;
    } else {
        std::cout << "Impossible d'écrire le fichier binaire" << std::endl;
    }
}

// check if any vector is a wall
bool AStar::isVectorValid(const std::vector<Node *> &vector) const {
    for (Node *n: vector) {
        if (n->x < 0 || n->x >= this->map->img->width() || n->y < 0 || n->y >= this->map->img->height() || n->isWall)
            return false;
    }
    return true;
}

std::vector<Node *> AStar::bresenham(Node *n1, Node *n2) const {
    std::vector<Node *> nodes;
    int x1 = n1->x;
    int y1 = n1->y;
    int x2 = n2->x;
    int y2 = n2->y;

    int delta_x(x2 - x1);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;

    int delta_y(y2 - y1);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;

    nodes.push_back(this->map->getNode(x1, y1));

    if (delta_x >= delta_y) {
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2) {
            if ((error > 0) || (!error && (ix > 0))) {
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;

            nodes.push_back(this->map->getNode(x1, y1));
        }
    } else {
        int error(delta_x - (delta_y >> 1));

        while (y1 != y2) {
            if ((error > 0) || (!error && (iy > 0))) {
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;

            nodes.push_back(this->map->getNode(x1, y1));
        }
    }
    return nodes;
}

std::vector<Node *> *AStar::lissage_naive(std::vector<Node *> *path) const {
    std::reverse(path->begin(), path->end());
    auto *newPath = new std::vector<Node *>();
    newPath->reserve(path->size());

    Node *currentNode = (*path)[0];

    for (int i = 1; i < path->size(); i++) {
        auto tmp = bresenham(currentNode, (*path)[i]);
        int tmp_j = i;
        for (int j = i + 1; j < path->size(); j++) {
            Node *nextNode = (*path)[j];
            auto vec_nodes = bresenham(currentNode, nextNode);
            if (isVectorValid(vec_nodes)) {
                tmp = vec_nodes;
                tmp_j = j;
            }
        }
        for (Node *n: tmp) {
            newPath->push_back(n);
        }
        currentNode = (*path)[tmp_j];
        i = tmp_j;
    }
    return newPath;
}

std::vector<Node *> *AStar::lissage(std::vector<Node *> *path) const {
    std::reverse(path->begin(), path->end());
    auto *newPath = new std::vector<Node *>();
    newPath->reserve(path->size());
    newPath->push_back((*path)[0]);

    Node *currentNode = (*path)[0];
    int next_i = 1;
    for (int i = 1; i < path->size(); i++) {
        Node *nextNode = (*path)[i];

        // Si le on peut tracer une ligne entre les deux noeuds
        // alors on essaye sur le suivant sinon on retourne sur le précédent
        auto vec_nodes = bresenham(currentNode, nextNode);
        while (isVectorValid(vec_nodes)) {
            if (next_i + 1 < path->size()) {
                // On test avec le suivant
                nextNode = (*path)[next_i + 1];
                vec_nodes = bresenham(currentNode, nextNode);

                // et on garde une trace de l'indice du noeud
                next_i++;
            } else {
                for (Node *n: vec_nodes) {
                    newPath->push_back(n);
                }
                std::reverse(newPath->begin(), newPath->end());
                return newPath;
            }
        }

        nextNode = (*path)[next_i - 1];
        vec_nodes = bresenham(currentNode, nextNode);

        // On ajoute les noeuds de la ligne droite dans la nouvelle liste
        for (Node *n: vec_nodes) {
            newPath->push_back(n);
        }
        i = next_i - 1;
        currentNode = nextNode;
    }
    std::reverse(newPath->begin(), newPath->end());
    return newPath;
}
