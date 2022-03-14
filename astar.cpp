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
            reconstructPath(currentNode);
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
        printf("Ecriture reussie\n");
        file.close();
        std::cout << "Ecriture du fichier binaire reussie" << std::endl;
    } else {
        std::cout << "Impossible d'ecrire le fichier binaire" << std::endl;
    }
}