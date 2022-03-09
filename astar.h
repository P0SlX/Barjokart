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
    const Pair src;
    std::vector<Pair> dest;
    const std::string filename;
    cimg_library::CImg<unsigned char> imageObject;

public:
    AStar(Pair &src, const int dest[3], std::string &filename);

    bool isValid(const Pair &point) const;

    bool isUnBlocked(const Pair &point) const;

    double heuristic(const Pair &source) const;

    std::vector<Pair> *tracePath(Pair &d);

    static std::vector<Pair> *speedVector(std::vector<Pair> *path);

    std::vector<Pair> *aStarSearch();

    static void writeFile(std::vector<Pair>);
};


#endif //BARJOKART_ASTAR_H
