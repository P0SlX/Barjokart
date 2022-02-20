#include "image.h"

Image::Image(std::string path) {
    this->path = path;
    this->image = cimg_library::CImg<float>(path.c_str());
    this->width = this->image.width();
    this->height = this->image.height();
}

Image::~Image() {
    this->image.~CImg<float>();
}

bool Image::isVectorValid(std::vector<Point> &vec) const {
    // Sanity check
    if (vec.empty())
        return false;

    for (auto &coordVec: vec) {
        // Check if the point coords are valid
        if (coordVec.x < 0 || coordVec.x >= this->width ||
            coordVec.y < 0 || coordVec.y >= this->height)
            return false;

        Pixel pixelColors = this->getPixelColors(coordVec);

        if (pixelColors.r == 0 && pixelColors.g == 0 && pixelColors.b == 0)
            return false;
    }
    return true;
}

std::vector<Point> Image::getVectorFromBresenham(Point &p1, Point &p2) const {
    // Check if points are valid
    if (p1.x < 0 || p1.x >= this->width ||
        p1.y < 0 || p1.y >= this->height ||
        p2.x < 0 || p2.x >= this->width ||
        p2.y < 0 || p2.y >= this->height)
        return {};

    std::vector<Point> vec;
    const bool steep = (std::abs(p2.y - p1.y) > std::abs(p2.x - p1.x));

    if (steep) {
        std::swap(p1.x, p1.y);
        std::swap(p2.x, p2.y);
    }

    if (p1.x > p2.x) {
        std::swap(p1.x, p2.x);
        std::swap(p1.y, p2.y);
    }

    const float dx = (float) p2.x - (float) p1.x;
    const float dy = std::abs((float) p2.y - (float) p1.y);

    float err = dx / 2.0f;
    const int ystep = (p1.y < p2.y) ? 1 : -1;
    int y = p1.y;

    const int maxX = p2.x;

    for (int x = p1.x; x < maxX; ++x) {
        if (steep)
            vec.emplace_back(y, x);
        else
            vec.emplace_back(x, y);

        err -= dy;
        if (err < 0) {
            y += ystep;
            err += dx;
        }
    }
    return vec;
}

std::vector<Point> Image::aStar(Point &p1, Point &p2) const {
    // A* algorithm
    std::vector<Point> vec;
}


