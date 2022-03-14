#ifndef BARJOKART_ASTAR_H
#define BARJOKART_ASTAR_H

#include <array>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include "CImg.h"
#include <fstream>
#include "string"
#include <vector>
#include <bitset>
#include "map.h"

class AStar {
public:
    Node *startNode;
    std::vector<Node *> openList, closedList;
    std::vector<Node *> *path;
    Map *map;

    AStar(Pair start, Map *map);

    ~AStar();

    void reconstructPath(Node *endNode);

    double heuristic(Node *node, Node *dest);

    std::vector<Node *> *aStarSearch();

    std::vector<Pair> *nodesToSpeedVector(std::vector<Node *> *nodes);

    static void writeFile(std::vector<Pair> &vector, const std::string &filename);

    void pushOpen(Node *node) {
        this->openList.push_back(node);
        std::push_heap(this->openList.begin(), this->openList.end(), compareNodes());
        node->isOpen = true;
    }

    void popOpen(Node *node) {
        std::pop_heap(this->openList.begin(), this->openList.end(), compareNodes());
        this->openList.pop_back();
        node->isOpen = false;
    }

    struct compareNodes {
        bool operator()(const Node *s1, const Node *s2) const {
            return s1->f < s2->f;
        }
    };
};

#endif //BARJOKART_ASTAR_H