#ifndef BARJOKART_IMAGE_HPP
#define BARJOKART_IMAGE_HPP


#include <string>
#include <vector>
#include "CImg.h"

struct point
{
    int x;
    int y;
};

struct pixel
{
    float r;
    float g;
    float b;
};


class Image
{
private:
    int width;
    int height;
    cimg_library::CImg<float> image;
    std::string path;

public:
    explicit Image(std::string path);

    ~Image();

    /**
     * @details Check si le vecteur de pixels est valide (s'il n'y a aucun pixel noir)
     * @details Le vecteur est sous la sorte [[x,y],[x,y],...]
     * @return bool
     */
    [[nodiscard]] bool isVectorValid(std::vector<point> vec) const;

    [[nodiscard]] std::vector<point> getVectorFromBresenham(point p1, point p2) const;

    [[nodiscard]] std::vector<point> aStart(point p1, point p2) const;

    [[nodiscard]] pixel getPixelColors(point p) const
    {
        return pixel{image(p.x, p.y, 0), image(p.x, p.y, 1), image(p.x, p.y, 2)};
    }

    [[nodiscard]] float getPixelRed(point p) const
    {
        return this->image(p.x, p.y, 0, 0);
    }

    [[nodiscard]] float getPixelGreen(point p) const
    {
        return this->image(p.x, p.y, 0, 1);
    }

    [[nodiscard]] float getPixelBlue(point p) const
    {
        return this->image(p.x, p.y, 0, 2);
    }

    [[nodiscard]] int getWidth() const
    {
        return width;
    }

    [[nodiscard]] int getHeight() const
    {
        return height;
    }

    [[nodiscard]] std::string getPath() const
    {
        return path;
    }

    cimg_library::CImg<float> getImage()
    {
        return image;
    }
};


#endif //BARJOKART_IMAGE_HPP
