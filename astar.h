#ifndef BARJOKART_ASTAR_H
#define BARJOKART_ASTAR_H

#include <vector>
#include <queue>
#include <iostream>

struct Node {
    int x, y, cout, heuristique, f;
    Node *parent;

    Node(int x, int y, int cout, int heuristique, int f, Node *parent) : x(x), y(y), cout(cout),
                                                                         heuristique(heuristique), f(f),
                                                                         parent(parent) {}
};

std::vector<Node *> reconstruct_path(Node *pNode);

int heuristic(Node *node, Node *goal);

std::vector<Node *> get_neighbors(Node *node, std::vector<std::vector<Node *> > &grid);

void print_grid_path(std::vector<std::vector<Node *> > &grid, std::vector<Node *> path);

std::vector<Node *> a_star(std::vector<std::vector<Node *> > &grid, Node *start, Node *end);


template<class ADAPTER>
typename ADAPTER::container_type &get_container(ADAPTER &a);


#endif //BARJOKART_ASTAR_H
