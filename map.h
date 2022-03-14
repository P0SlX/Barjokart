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


    Map(cimg_library::CImg<unsigned char> *img, const unsigned char *color_dest);

    ~Map();

    void print();

    // !!! Les arguments sont inversés
    Node *getNode(int x, int y) {
        return map[x * img->height() + y];
    }

};

#endif //BARJOKART_MAP_H
