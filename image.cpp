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

bool Image::isVectorValid(std::vector<std::vector<int> > vec) const
{
    // Sanity check
    if (vec.empty())
        return false;

    for (auto &coordVec: vec)
    {
        if (coordVec.empty())
            return false;

        std::vector<float> colors = this->getPixelColors(coordVec);

        if (colors[0] == 0 && colors[1] == 0 && colors[2] == 0)
            return false;
    }
    return true;
}

std::vector<std::vector<int> > Image::getVectorFromBresenhamV1(int x1, int y1, int x2, int y2) const
{
    std::vector<std::vector<int> > vec;
    const bool steep = (std::abs(y2 - y1) > std::abs(x2 - x1));

    if (steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if (x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = std::abs(y2 - y1);

    float err = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = y1;

    const int maxX = x2;

    for (int x = x1; x < maxX; ++x)
    {
        if (steep)
            vec.push_back({y, x});
        else
            vec.push_back({x, y});

        err -= dy;
        if (err < 0)
        {
            y += ystep;
            err += dx;
        }
    }
    return vec;
}

std::vector<std::vector<int> > Image::aStart(int x1, int y1, int x2, int y2) const {
    // A* algorithm

    std::vector<std::vector<int> > nodeList;
    int nodeListIndex = 0;
    std::vector<std::vector<int> > pathList;
    // path list index
    int pathListIndex = 0;
    bool found = false;
    bool path = false;

}


