#include "astar.h"

template<class ADAPTER>
typename ADAPTER::container_type &get_container(ADAPTER &a) {
    struct hack : ADAPTER {
        static typename ADAPTER::container_type &get(ADAPTER &a) {
            return a.*&hack::c;
        }
    };
    return hack::get(a);
}


std::vector<Node *> reconstruct_path(Node *pNode) {
    std::vector<Node *> path;
    while (pNode != nullptr) {
        path.push_back(pNode);
        pNode = pNode->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int heuristic(Node *node, Node *goal) {
    return abs(node->x - goal->x) + abs(node->y - goal->y);
}

std::vector<Node *> get_neighbors(Node *node, std::vector<std::vector<Node *> > &grid) {
    std::vector<Node *> neighbors;
    // get all 8 neighbors
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) {
                continue;
            }
            int x = node->x + i;
            int y = node->y + j;
            if (x >= 0 && x < 10 && y >= 0 && y < 10) {
                neighbors.push_back(grid[x][y]);
            }
        }
    }
    return neighbors;
}

std::vector<Node *> a_star(std::vector<std::vector<Node *> > &grid, Node *start, Node *end) {
    std::vector<Node *> closedList;
    std::priority_queue<Node *, std::vector<Node *>, decltype(&heuristic)> openList(&heuristic);
    openList.push(start);
    while (!openList.empty()) {
        Node *u = openList.top();
        openList.pop();
        closedList.push_back(u);

        // Arrivé
        if (u->x == end->x && u->y == end->y) {
            return reconstruct_path(u);
        }

        std::vector<Node *> neighbors = get_neighbors(u, grid);

        for (auto v: neighbors) {
            if (std::find(closedList.begin(), closedList.end(), v) != closedList.end()) {
                continue;
            }
            v->cout = u->cout + 1;
            v->heuristique = heuristic(v, end);
            v->f = v->cout + v->heuristique;
            v->parent = u;

            if (std::find(get_container(openList).begin(), get_container(openList).end(), v) !=
                get_container(openList).end()) {
                if (v->f < get_container(openList).at(get_container(openList).size() - 1)->f) {
                    get_container(openList).erase(
                            std::remove(get_container(openList).begin(), get_container(openList).end(), v),
                            get_container(openList).end());
                    openList.push(v);
                }
            } else {
                openList.push(v);
            }
        }
    }
    return {};
}

void print_grid_path(std::vector<std::vector<Node *> > &grid, std::vector<Node *> path) {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            // if grid[i][j] is in path, print 'x'
            if (std::find(path.begin(), path.end(), grid[i][j]) != path.end()) {
                std::cout << "x";
            } else {
                std::cout << ".";
            }
        }
        std::cout << std::endl;
    }
}