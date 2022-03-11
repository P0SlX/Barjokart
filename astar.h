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
    bool is_wall;

    Node() : parent(-1, -1), f(-1), g(-1), h(-1) {}
};

class AStar {
private:
    int width, height;
    const Pair src;
    std::vector<Pair> dest;
    const std::string filename;
    std::vector<std::vector<Node *> > *grid;
    cimg_library::CImg<unsigned char> *img;

public:
    AStar(Pair &src, const int dest[3], std::string &filename);

    ~AStar();

    bool isValid(const Pair &point) const;

    bool isUnBlocked(const Pair &point) const;

    double heuristic(const Pair &source) const;

    std::vector<Node*> *tracePath(Pair &d);

    static std::vector<Pair> *speedVector(std::vector<Node*> *d);

    std::vector<Node*> *aStarSearch();

    static void writeFile(std::vector<Pair> *vecteur);
};


#endif //BARJOKART_ASTAR_H
