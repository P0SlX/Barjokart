#include "node.h"

Node::Node(int x, int y) {
    this->x = x;
    this->y = y;
    this->g = -1;
    this->h = -1;
    this->f = -1;
    this->parent = nullptr;
    this->isWall = false;
    this->isDestination = false;
    this->isOpen = false;
    this->isClosed = false;
    this->neighbors = new std::vector<std::pair<Node *, double> >();
}

Node::~Node() {
    this->parent = nullptr;
    delete this->neighbors;
    this->neighbors = nullptr;
}
