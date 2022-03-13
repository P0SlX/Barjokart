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


    Map(cimg_library::CImg<unsigned char> *img, unsigned char *color_dest);

    ~Map();

    void print();

    double heuristic(Node *node, Node *dest);

    void writePathToPNG(std::vector<Node *> *path, std::string filename);

    // !!! Les arguments sont inversés
    Node *getNode(int x, int y) {
        return map[y * img->width() + x];
    }

    bool isValid(int x, int y) {
        return x >= 0 && x < img->width() && y >= 0 && y < img->height();
    }
};

#endif //BARJOKART_MAP_H
