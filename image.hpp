#ifndef BARJOKART_IMAGE_HPP
#define BARJOKART_IMAGE_HPP


#include <string>
#include <vector>
#include "CImg.h"
#include "pixel.h"
#include "point.hpp"


class Image {
public:
    int width, height;
    cimg_library::CImg<float> image;
    std::string path;

    explicit Image(std::string path);

    ~Image();

    /**
     * @details Check si le vecteur de pixels est valide (s'il n'y a aucun pixel noir)
     * @details Le vecteur est de la sorte [[x,y],[x,y],...]
     * @return bool
     */
    bool isVectorValid(std::vector<Point> &vec) const;

    /**
     * @details Retourne un vecteur de pixels entre deux points
     * @details Les deux points doivent être valides
     * @return std::vector<Point>
     */
    std::vector<Point> getVectorFromBresenham(Point &p1, Point &p2) const;

    std::vector<Point> aStar(Point &p1, Point &p2) const;

    Pixel getPixelColors(Point &p) const {
        return Pixel{(int) image(p.x, p.y, 0), (int) image(p.x, p.y, 1), (int) image(p.x, p.y, 2)};
    }
};


#endif //BARJOKART_IMAGE_HPP
