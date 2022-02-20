#include "CImg.h"
#include <iostream>
#include "image.h"

using namespace cimg_library;

int main()
{
    auto *image = new Image("../hills.png");
    std::vector<int> start_coords = {89, 92};
    std::vector<int> end_color = {255, 0, 0};

    cimg_library::CImgDisplay display(image->image);
    std::cin.ignore();
}
