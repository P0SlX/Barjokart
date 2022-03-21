#ifndef BARJOKART_NODE_H
#define BARJOKART_NODE_H

#include "vector"

class Node {
public:
    int x, y;
    Node *parent;
    double f, g, h;
    bool isWall, isDestination, isOpen, isClosed;
    std::vector<std::pair<Node *, double> > *neighbors;

    Node(int x, int y);

    ~Node();

    int getCost(Node *node) const {
        return this->f;
    }

    int getDistanceFromStartNode(Node *node) const {
        return this->g;
    }

    int getHeuristic(Node *node) const {
        return this->h;
    }

    void setParent(Node *node) {
        this->parent = node;
    }

    void setCost(int f) {
        this->f = f;
    }

    void setDistanceFromStartNode(int g) {
        this->g = g;
    }

    void setHeuristic(int h) {
        this->h = h;
    }
};


#endif //BARJOKART_NODE_H
