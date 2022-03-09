#include "astar.h"

int main() {
    // TODO
    //  - TOML support for src and dest
    //  - Use of pointers for A*

    // start coord
   Pair src = Pair(89, 92);

    // destination color
    int dest[3] = {255, 0, 0};

    std::string filename;
    std::cout << "Entrez le nom du fichier paramètres : " << std::endl;
    std::cin >> filename;
    filename = "../param_circuits/" + filename;
    printf("%s\n", filename.c_str());
    /*toml::table tbl;

    try {
        tbl = toml::parse_file(filename);
    }
    catch (const toml::parse_error& e) {
        printf("Erreur de chargement\n");
    }*/
    
    std::string path = "hills.png";

    AStar astar(src, dest, path);
    astar.aStarSearch();

    std::vector<Pair> v1;
    Pair src1 = Pair(2, 2);
    v1.push_back(src);
    v1.push_back(src1);
    AStar::writeFile(v1);



    return 0;
}
