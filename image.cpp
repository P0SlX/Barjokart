#include <cassert>
#include "image.h"

Image::Image(std::string path) {
    this->path = path;
    this->imageObject = cimg_library::CImg<unsigned char>(path.c_str());

    // Fill the image matrix with the image data
    for (auto y = 0; y < imageObject.height(); y++) {
        std::vector<Point> row;
        row.reserve(imageObject.width());
        for (auto x = 0; x < imageObject.width(); x++) {
            row.emplace_back(x, y,
                             (int) imageObject(x, y, 0),
                             (int) imageObject(x, y, 1),
                             (int) imageObject(x, y, 2));
        }
        img.push_back(row);
    }

    this->width = this->imageObject.width();
    this->height = this->imageObject.height();
}

bool Image::isVectorValid(std::vector<Point> &vec) const {
    // Sanity check
    assert(!vec.empty());

    for (auto &pt: vec) {
        // Check if the point coords are valid
        if (pt.x < 0 || pt.x >= this->width ||
            pt.y < 0 || pt.y >= this->height)
            return false;

        if (pt.r == 0 && pt.g == 0 && pt.b == 0)
            return false;
    }
    return true;
}

std::vector<Point> Image::getVectorFromBresenham(Point p1, Point p2) const {
    // Check if points are valid
    assert(p1.x >= 0 && p1.x <= this->width &&
           p1.y >= 0 && p1.y <= this->height &&
           p2.x >= 0 && p2.x <= this->width &&
           p2.y >= 0 && p2.y <= this->height);

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
            vec.emplace_back(this->img[x][y]);
        else
            vec.emplace_back(this->img[y][x]);

        err -= dy;
        if (err < 0) {
            y += ystep;
            err += dx;
        }
    }
    return vec;
}

std::vector<Point> Image::aStar(Point &p1, Point &p2) const {
    std::vector<Point> solution;
}

void Image::print() const {
    for (int j = 0; j < this->width; ++j) {
        for (int i = 0; i < this->height; ++i) {
            if (this->img[j][i].r == 0 && this->img[j][i].g == 0 && this->img[j][i].b == 0)
                std::cout << "M";
            else if (this->img[j][i].r == 255 && this->img[j][i].g == 255 && this->img[j][i].b == 255)
                std::cout << " ";
            else
                std::cout << "/";
        }
        std::cout << std::endl;
    }
}

Point Image::operator()(int x, int y) const {
    assert(x >= 0 && x <= this->width && y >= 0 && y <= this->height);
    return this->img[y][x];
}

Point &Image::operator()(int x, int y) {
    assert(x >= 0 && x <= this->width && y >= 0 && y <= this->height);
    return this->img[y][x];
}



