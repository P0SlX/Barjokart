#include "image.hpp"

Image::Image(std::string path)
{
    this->path = path;
    this->image = cimg_library::CImg<float>(path.c_str());
    this->width = this->image.width();
    this->height = this->image.height();
}

Image::~Image()
{
    this->image.~CImg<float>();
}

bool Image::isVectorValid(std::vector<point> vec) const
{
    // Sanity check
    if (vec.empty())
        return false;

    for (auto &coordVec: vec)
    {
        pixel pixelColors = this->getPixelColors(coordVec);

        if (pixelColors.r == 0 && pixelColors.g == 0 && pixelColors.b == 0)
            return false;
    }
    return true;
}

std::vector<point > Image::getVectorFromBresenham(point p1, point p2) const
{
    std::vector<point > vec;
    const bool steep = (std::abs(p2.y - p1.y) > std::abs(p2.x - p1.x));

    if (steep)
    {
        std::swap(p1.x, p1.y);
        std::swap(p2.x, p2.y);
    }

    if (p1.x > p2.x)
    {
        std::swap(p1.x, p2.x);
        std::swap(p1.y, p2.y);
    }

    const float dx = p2.x - p1.x;
    const float dy = std::abs(p2.y - p1.y);

    float err = dx / 2.0f;
    const int ystep = (p1.y < p2.y) ? 1 : -1;
    int y = p1.y;

    const int maxX = p2.x;

    for (int x = p1.x; x < maxX; ++x)
    {
        if (steep)
            vec.push_back(point{y, x});
        else
            vec.push_back(point{x, y});

        err -= dy;
        if (err < 0)
        {
            y += ystep;
            err += dx;
        }
    }
    return vec;
}

std::vector<point> Image::aStart(point p1, point p2) const {
    // A* algorithm
    std::vector<point> vec;
}


