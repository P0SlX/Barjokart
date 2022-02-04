#ifndef BARJOKART_IMAGE_HPP
#define BARJOKART_IMAGE_HPP


#include <string>
#include <vector>
#include "CImg.h"

struct coord {
    int x;
    int y;
};

struct pixel {
    unsigned char r;
    unsigned char g;
    unsigned char b;
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
    [[nodiscard]] bool isVectorValid(std::vector<coord > vec) const;

    [[nodiscard]] std::vector<coord> getVectorFromBresenhamV1(int x1, int y1, int x2, int y2) const;

    [[nodiscard]] std::vector<coord > aStart(int x1, int y1, int x2, int y2) const;

    [[nodiscard]] std::vector<float> getPixelColors(int x, int y) const
    {
        std::vector<float> colors;
        colors.push_back(image(x, y, 0, 0));
        colors.push_back(image(x, y, 0, 1));
        colors.push_back(image(x, y, 0, 2));
        return colors;
    }

    [[nodiscard]] std::vector<float> getPixelColors(std::vector<int> vec) const
    {
        std::vector<float> colors;
        colors.push_back(image(vec[0], vec[1], 0, 0));
        colors.push_back(image(vec[0], vec[1], 0, 1));
        colors.push_back(image(vec[0], vec[1], 0, 2));
        return colors;
    }

    [[nodiscard]] float getPixelRed(int x, int y) const
    {
        return this->image(x, y, 0, 0);
    }

    [[nodiscard]] float getPixelGreen(int x, int y) const
    {
        return this->image(x, y, 0, 1);
    }

    [[nodiscard]] float getPixelBlue(int x, int y) const
    {
        return this->image(x, y, 0, 2);
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
