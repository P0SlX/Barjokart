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

typedef std::pair<int, int> Pair;
typedef std::tuple<double, int, int> Tuple;

struct Node {
    Pair parent;
    double f, g, h;

    Node() : parent(-1, -1), f(-1), g(-1), h(-1) {}
};

class AStar {
private:
    int width, height;
    std::vector<std::vector<int> > grid;
    std::vector<std::vector<Node> > nodeDetails;
    const Pair src, dest;
    const std::string filename;

public:
    AStar(Pair start, Pair dest, std::string filename);

    bool isValid(const Pair &point) const;

    bool isUnBlocked(const Pair &point) const;

    double heuristic(const Pair &source) const;

    void tracePath();

    void aStarSearch();
};


#endif //BARJOKART_ASTAR_H
