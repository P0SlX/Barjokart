#include "CImg.h"
#include <iostream>
#include "image.h"

using namespace cimg_library;

int main() {

    // Avec Cmake il faut faire ../ car c'est build dans cmake-build-debug et le png est à la racine
    Image image = Image("../hills.png");

    // Affichage de l'image
    image.print();

    std::vector<int> start_coords = {89, 92};
    std::vector<int> end_color = {255, 0, 0};

    return 0;
}
