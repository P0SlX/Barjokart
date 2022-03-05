#include "astar.h"

AStar::AStar(Pair src, Pair dest, std::string filename) : src(std::move(src)), dest(std::move(dest)), filename(std::move(filename)) {
    cimg_library::CImg<unsigned char> imageObject = cimg_library::CImg<unsigned char>(this->filename.c_str());

    this->height = imageObject.height();
    this->width = imageObject.width();

    // Initialize the map
    this->grid.reserve(this->height);
    for (int i = 0; i < this->height; i++) {
        this->grid.emplace_back(this->width);
    }

    // Transforming the image into a grid of 0s and 1s
    for (int i = 0; i < this->height; i++) {
        for (int j = 0; j < this->width; j++) {
            if (imageObject(j, i, 0) == 0 && imageObject(j, i, 1) == 0 && imageObject(j, i, 2) == 0) {
                this->grid[j][i] = 0;
            } else {
                this->grid[j][i] = 1;
            }
        }
    }

    // Initialize the open list
    this->nodeDetails.reserve(this->height);
    for (int i = 0; i < this->height; i++) {
        this->nodeDetails.emplace_back(this->width);
    }
}

bool AStar::isValid(const Pair &point) const {
    if (this->height > 0 && this->width > 0)
        return (point.second >= 0) && (point.second < this->height) && (point.first >= 0) && (point.first < this->width);
    return false;
}

bool AStar::isUnBlocked(const Pair &point) const {
    return isValid(point) && grid[point.first][point.second] == 1;
}

double AStar::heuristic(const Pair &source) const {
    return (abs(source.first - this->dest.first) + abs(source.second - this->dest.second));
}

void AStar::tracePath() {
    std::vector<Pair> path;

    int i = this->dest.first, j = this->dest.second;
    Pair next_node = this->nodeDetails[j][i].parent;
    do {
        path.push_back(next_node);
        next_node = this->nodeDetails[j][i].parent;
        i = next_node.first;
        j = next_node.second;
    } while (this->nodeDetails[j][i].parent != next_node);

    path.emplace_back(i, j);
    path.push_back(dest);

    for (int k = 0; k < this->height; k++) {
        for (int l = 0; l < this->width; l++) {
            if (find(path.begin(), path.end(), Pair(l, k)) != path.end()) {
                std::cout << "X ";
                continue;
            }
            if (this->grid[l][k] == 1)
                std::cout << "  ";
            else
                std::cout << "██";
        }
        // \n pour pas flush le buffer et gagner du temps
        std::cout << "\n";
    }
}

void AStar::aStarSearch() {
    if (!isValid(this->src)) {
        printf("Le point source n'est pas dans l'image\n");
        return;
    }

    if (!isValid(this->dest)) {
        printf("Le point de destination n'est pas dans l'image\n");
        return;
    }

    if (!isUnBlocked(this->src)) {
        printf("Le point source est un obstacle\n");
        return;
    }

    if (!isUnBlocked(this->dest)) {
        printf("Le point de destination est un obstacle\n");
        return;
    }


    if (!isUnBlocked(this->src) || !isUnBlocked(this->dest)) {
        printf("Le point source et/ou destination est(sont) bloqué(s)\n");
        return;
    }

    if (this->src == this->dest) {
        printf("Le point source est le même que celui de destination\n");
        return;
    }

    bool closedList[this->height][this->width];
    memset(closedList, false, sizeof(closedList));

    int i, j;
    i = src.first, j = src.second;
    this->nodeDetails[j][i].f = 0.0;
    this->nodeDetails[j][i].g = 0.0;
    this->nodeDetails[j][i].h = 0.0;
    this->nodeDetails[j][i].parent = {i, j};

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
                    if (neighbour == dest) {
                        this->nodeDetails[neighbour.second][neighbour.first].parent = {i, j};
                        printf("Le point de destination à été atteint\n");
                        tracePath();
                        return;
                    } else if (!closedList[neighbour.second][neighbour.first] && isUnBlocked(neighbour)) {
                        double gNew = this->nodeDetails[j][i].g + 1.0;
                        double hNew = heuristic(neighbour);
                        double fNew = gNew + hNew;

                        if (this->nodeDetails[neighbour.second][neighbour.first].f == -1 ||
                                this->nodeDetails[neighbour.second][neighbour.first].f > fNew) {
                            openList.emplace(fNew, neighbour.second, neighbour.first);

                            this->nodeDetails[neighbour.second][neighbour.first].g = gNew;
                            this->nodeDetails[neighbour.second][neighbour.first].h = hNew;
                            this->nodeDetails[neighbour.second][neighbour.first].f = fNew;
                            this->nodeDetails[neighbour.second][neighbour.first].parent = {i, j};
                        }
                    }
                }
            }
        }
    }
    printf("Impossible de trouver le chemin\n");

}

