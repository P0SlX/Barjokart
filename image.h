#ifndef BARJOKART_IMAGE_H
#define BARJOKART_IMAGE_H


#include <string>
#include <vector>
#include "CImg.h"
#include <iostream>
#include "point.h"


class Image {
public:
    int width, height;
    cimg_library::CImg<unsigned char> imageObject;
    std::string path;
    std::vector<std::vector<Point> > img;

    explicit Image(std::string path);

    ~Image() = default;

    /**
     * @details Check si le vecteur de pixels est valide (s'il n'y a aucun pixel noir)
     * @return bool
     */
    bool isVectorValid(std::vector<Point> &vec) const;

    /**
     * @details Utilise l'algo de breseham pour trouver les points entre deux points
     * @details Les deux points doivent être valides
     * @details Les points sont passées par copie pour ne pas affecter les points de l'image
     * @return std::vector<Point>
     */
    [[nodiscard]] std::vector<Point> getVectorFromBresenham(Point p1, Point p2) const;

    std::vector<Point> aStar(Point &p1, Point &p2) const;

    Point operator()(int x, int y) const;

    Point &operator()(int x, int y);

    void print() const;
};


#endif //BARJOKART_IMAGE_H
