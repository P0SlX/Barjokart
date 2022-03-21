#ifndef BARJOKART_MAP_H
#define BARJOKART_MAP_H

#include "CImg.h"
#include <iostream>
#include "node.h"

typedef std::pair<int, int> Pair;

class Map {
public:
    std::vector<Node *> map;
    cimg_library::CImg<unsigned char> *img;
    Node *destination;
    std::vector<Node *> destinationsNodes;
    int acc_max;

    Map(cimg_library::CImg<unsigned char> *img, const unsigned char *color_dest, int acc_max);

    ~Map();

    void print();

    Node *getNode(int x, int y) {
        return map[x * img->height() + y];
    }

};

#endif //BARJOKART_MAP_H
