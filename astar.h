#ifndef BARJOKART_ASTAR_H
#define BARJOKART_ASTAR_H

#include <fstream>
#include <chrono>

#include "CImg.h"
#include "map.h"

class AStar {
public:
    Node *startNode;
    std::vector<Node *> openList, closedList;
    std::vector<Node *> *path;
    Map *map;

    AStar(Pair start, Map *map);

    ~AStar();

    void reconstructPath(Node *endNode) const;

    static double heuristic(Node *node, Node *dest);

    std::vector<Node *> *aStarSearch();

    static std::vector<Pair> *nodesToSpeedVector(std::vector<Node *> *nodes);

    static void writeFile(std::vector<Pair> &vector, const std::string &filename);

    [[nodiscard]] bool isVectorValid(const std::vector<Node *> &vector) const;

    std::vector<Node *> bresenham(Node *n1, Node *n2) const;

    std::vector<Node *> bresenham2(Node *n1, Node *n2) const;

    std::vector<Node *> *lissage(std::vector<Node *> *vecPath) const;

    std::vector<Node *> *lissage_naive(std::vector<Node *> *vecPath) const;

    std::vector<Node *> *acceleration(std::vector<Node *> *vecPath) const;

    static std::vector<Pair> *cutting(std::vector<Pair> &vectorPath) ;

    std::vector<Pair> *acceleration2(std::vector<Pair> &vectorPath, std::vector<Pair> &vecteur2);


    // Push sur la liste openList et sur la pile
    void pushOpen(Node *node) {
        this->openList.push_back(node);
        std::push_heap(this->openList.begin(), this->openList.end(), compareNodes());
        node->isOpen = true;
    }

    // Pop le dernier élément d'openList et le premier de la pile
    void popOpen(Node *node) {
        std::pop_heap(this->openList.begin(), this->openList.end(), compareNodes());
        this->openList.pop_back();
        node->isOpen = false;
    }

    // Petite struct qui permet facilement de comparer les noeuds
    struct compareNodes {
        bool operator()(const Node *s1, const Node *s2) const {
            return s1->f < s2->f;
        }
    };
};

#endif //BARJOKART_ASTAR_H