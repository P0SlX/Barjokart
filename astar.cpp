#include "astar.h"

AStar::AStar(Pair start, Map *map) {
    this->startNode = map->getNode(start.first, start.second);
    this->startNode->f = this->startNode->g = this->startNode->h = 0;
    this->path = new std::vector<Node *>();
    this->map = map;
}

AStar::~AStar() {
    delete this->path;
    this->path = nullptr;
}

double AStar::heuristic(Node *node, Node *dest) {
    // Calcule la distance Euclidienne entre 2 points
    double x_square = (node->x - dest->x) * (node->x - dest->x);
    double y_square = (node->y - dest->y) * (node->y - dest->y);
    return sqrt(x_square + y_square);
}

void AStar::reconstructPath(Node *endNode) const {
    // Reconstruit le chemin en partant de la fin
    Node *parent = endNode->parent;
    this->path->push_back(endNode);
    while (parent != nullptr) {
        path->push_back(parent);
        parent = parent->parent;
    }
    // Inverse le chemin pour avoir le chemin de départ à l'arrivée
    std::reverse(path->begin(), path->end());
}

std::vector<Node *> *AStar::aStarSearch() {
    std::cout << "A* en cours... ";
    std::cout.flush();
    auto start = std::chrono::high_resolution_clock::now();


    Node *currentNode = this->startNode;
    Node *neighborsNode;
    double f, g, h;

    // Transforme openlist en une pile afin d'avoir le meilleur noeud en tête
    std::make_heap(this->openList.begin(), this->openList.end(), compareNodes());
    pushOpen(currentNode);

    while (!this->openList.empty()) {
        std::sort(this->openList.begin(), this->openList.end(), compareNodes());

        // Récupère le meilleur noeud de la liste
        currentNode = this->openList.front();
        popOpen(currentNode);

        // Comme on a déjà vérifié le noeud, on le met dans la liste fermé
        currentNode->isClosed = true;
        this->closedList.push_back(currentNode);

        // Si le noeud est la destination, on reconstruit le chemin
        if (currentNode->isDestination) {
            auto stop = std::chrono::high_resolution_clock::now();
            double duration = (double) duration_cast<std::chrono::microseconds>(stop - start).count();
            duration /= 1000;
            reconstructPath(currentNode);
            std::cout << "Terminé en " << duration << "ms." << std::endl;
            return this->path;
        }

        // Cette boucle est très longue si il y a beaucoup de noeuds de destination
        // Mais c'est la seule facon de calculer le le noeud d'arrivé le plus près
        double h_min = std::numeric_limits<double>::max();
        for (Node *n: this->map->destinationsNodes) {
            double tmp = heuristic(currentNode, n);
            if (tmp < h_min) {
                h_min = tmp;
                this->map->destination = n;
            }
        }

        // Parcours les noeuds voisins
        for (const auto &neighbors: *currentNode->neighbors) {
            neighborsNode = neighbors.first;

            // Si c'est un mur on passe au suivant on ne calcule rien
            if (neighborsNode->isWall)
                continue;

            // Addition du coût du noeud courant au coût du noeud voisin
            g = currentNode->g + neighbors.second;

            // Si le noeud a été vérifié et que le coût est plus petit que le coût actuel on le passe car ca nous intéresse pas
            if ((neighborsNode->isOpen || neighborsNode->isClosed) && neighborsNode->g <= g)
                continue;

            // On va calculer sa distance au points de destination et calculer f en fonction de ce coût et de la distance
            // Puis on attribue le noeud parent afin de reconstruire le chemin plus tard
            h = this->heuristic(neighborsNode, this->map->destination);
            f = g + h;
            neighborsNode->f = f;
            neighborsNode->g = g;
            neighborsNode->h = h;
            neighborsNode->setParent(currentNode);

            if (!neighborsNode->isOpen)
                pushOpen(neighborsNode);
        }
    }
    return nullptr;
}

std::vector<Pair> *AStar::nodesToSpeedVector(std::vector<Node *> *path) {
    std::cout << "Transformation des noeuds en vecteur vitesse... ";
    std::cout.flush();
    // Transforme le chemin en un vecteur de vitesse pour l'écrire dans le fichier par la suite
    auto *speedVector = new std::vector<Pair>();
    for (int k = 0; k < path->size() - 1; k++) {
        speedVector->emplace_back((*path)[k + 1]->x - (*path)[k]->x, (*path)[k + 1]->y - (*path)[k]->y);
    }
    std::cout << "Terminé." << std::endl;
    return speedVector;
}

void AStar::writeFile(std::vector<Pair> &vector, const std::string &filename) {
    std::cout << "Ecriture du fichier de sortie... ";
    std::cout.flush();
    std::ofstream file;
    file.open(filename, std::ios::binary);
    if (file.is_open()) {
        // On écrit le vecteur de vitesse dans le fichier en binaire
        for (Pair p: vector) {
            file.write((char *) &p.first, sizeof(int));
            file.write((char *) &p.second, sizeof(int));
        }
        file.close();
        std::cout << "Terminé." << std::endl;
    } else {
        std::cout << "Impossible d'écrire le fichier binaire" << std::endl;
    }
}

bool AStar::isVectorValid(const std::vector<Node *> &vector) const {
    // Permet de check rapidement si le vecteur est valide
    // On vérifie que le vecteur est bien dans l'image et qu'il n'est pas un mur
    for (Node *n: vector) {
        if (n->x < 0 || n->x >= this->map->img->width() || n->y < 0 || n->y >= this->map->img->height() || n->isWall)
            return false;
    }
    return true;
}

std::vector<Node *> AStar::bresenham2(Node *n1, Node *n2) const {
    // CE N'EST PAS L'ALGORITHME QUE NOUS UTILISIONS, NOUS AVONS DU AJOUTER CA EN COMPLÉMENT DU NOTRE
    // POUR ÊTRE SUR QUE NOUS AYONS UN RÉSULTAT CORRECTE
    // Algoritme repris de https://gist.github.com/jngl/c70bbbe40d6d6bbf43979138fb7fd8bb
    std::vector<Node *> nodes;
    int x1 = n1->x;
    int y1 = n1->y;
    int x2 = n2->x;
    int y2 = n2->y;

    int dx = x2 - x1;
    int dy;

    if (dx != 0) {
        if (dx > 0) {
            dy = y2 - y1;
            if (dy != 0) {
                if (dy > 0) {
                    if (dx >= dy) {
                        int e = dx;
                        dx = e * 2;
                        dy = dy * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            ++x1;
                            if (x1 == x2) {
                                break;
                            }
                            e -= dy;
                            if (e < 0) {
                                ++y1;
                                e += dx;
                            }
                        }
                    } else {
                        int e = dy;
                        dy = e * 2;
                        dx = dx * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            ++y1;
                            if (y1 == y2) {
                                break;
                            }
                            e -= dx;
                            if (e < 0) {
                                ++x1;
                                e += dy;
                            }
                        }
                    }
                } else {
                    if (dx >= -dy) {
                        int e = dx;
                        dx = e * 2;
                        dy = dy * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            ++x1;
                            if (x1 == x2) {
                                break;
                            }
                            e = e + dy;

                            if (e < 0) {
                                --y1;
                                e = e + dx;
                            }
                        }
                    } else {
                        int e = dy;
                        dy = e * 2;
                        dx = dx * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            --y1;
                            if (y1 == y2) {
                                break;
                            }
                            e += dx;
                            if (e > 0) {
                                ++x1;
                                e += dy;
                            }
                        }
                    }
                }
            } else {
                do {
                    nodes.push_back(this->map->getNode(x1, y1));
                    ++x1;
                } while (x1 != x2);
            }
        } else {
            dy = y2 - y1;
            if (dy != 0) {
                if (dy > 0) {
                    if (-dx >= dy) {
                        int e = dx;
                        dx = e * 2;
                        dy = dy * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            --x1;
                            if (x1 == x2) {
                                break;
                            }
                            e += dy;
                            if (e >= 0) {
                                ++y1;
                                e += dx;
                            }
                        }
                    } else {
                        int e = dy;
                        dy = e * 2;
                        dx = dx * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            ++y1;
                            if (y1 == y2) {
                                break;
                            }
                            e += dx;
                            if (e <= 0) {
                                --x1;
                                e += dy;
                            }
                        }
                    }
                } else {
                    if (dx <= dy) {
                        int e = dx;
                        dx = e * 2;
                        dy = dy * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            --x1;
                            if (x1 == x2) {
                                break;
                            }
                            e -= dy;
                            if (e >= 0) {
                                --y1;
                                e += dx;
                            }
                        }
                    } else {
                        int e = dy;
                        dy = e * 2;
                        dx = dx * 2;
                        for (;;) {
                            nodes.push_back(this->map->getNode(x1, y1));
                            --y1;
                            if (y1 == y2) {
                                break;
                            }
                            e -= dx;
                            if (e >= 0) {
                                --x1;
                                e += dy;
                            }
                        }
                    }
                }
            } else {
                do {
                    nodes.push_back(this->map->getNode(x1, y1));
                    --x1;
                } while (x1 != x2);
            }
        }
    } else {
        dy = y2 - y1;
        if (dy != 0) {
            if (dy > 0) {
                do {
                    nodes.push_back(this->map->getNode(x1, y1));
                    ++y1;
                } while (y1 != y2);
            } else {
                do {
                    nodes.push_back(this->map->getNode(x1, y1));
                    --y1;
                } while (y1 != y2);
            }
        }
    }
    return nodes;
}

std::vector<Node *> AStar::bresenham(Node *n1, Node *n2) const {
    // Pseudo code implémenté en C++
    // C'est une implémentation qui ne perd pas en précision
    // et qui est plus rapide que les versions avec multiplications et divisions
    // Notamment dû au faut qu'un CPU calcule plus rapidement les additions et soustractions
    // que les multiplications et divisions
    std::vector<Node *> nodes;
    int x1 = n1->x;
    int y1 = n1->y;
    int x2 = n2->x;
    int y2 = n2->y;

    int delta_x(x2 - x1);
    int const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;

    int delta_y(y2 - y1);
    int const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;

    nodes.push_back(this->map->getNode(x1, y1));

    if (delta_x >= delta_y) {
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2) {
            if ((error > 0) || (!error && (ix > 0))) {
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;

            nodes.push_back(this->map->getNode(x1, y1));
        }
    } else {
        int error(delta_x - (delta_y >> 1));

        while (y1 != y2) {
            if ((error > 0) || (!error && (iy > 0))) {
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;

            nodes.push_back(this->map->getNode(x1, y1));
        }
    }
    return nodes;
}

std::vector<Node *> *AStar::lissage_naive(std::vector<Node *> *vecPath) const {
    // Lissage du chemin naive qui permet de tracer une droite avec le point le plus loin possible
    // Afin de faire une grande ligne droite et donc accélérer le kart le plus possible

    // Cette fonction est utilisé en complément d'une autre car elle n'est pas implémenté de la même manière
    // Celle-ci va juste essayer tout les points sans réfléchir et tracer une grande droite sur le dernier point
    // du chemin qui peut être tracé d'une droite.
    auto *newPath = new std::vector<Node *>();
    newPath->reserve(vecPath->size());

    Node *currentNode = (*vecPath)[0];

    // On commence par le premier point
    for (int i = 1; i < vecPath->size(); i++) {
        // On trace la droite entre le point courant et le point suivant
        auto tmp = bresenham(currentNode, (*vecPath)[i]);
        int tmp_j = i;

        // On cherche le point le plus loin du point courant permettant de tracer une droite valide
        for (int j = i + 1; j < vecPath->size(); j++) {
            Node *nextNode = (*vecPath)[j];
            auto vec_nodes = bresenham(currentNode, nextNode);
            if (isVectorValid(vec_nodes)) {
                tmp = vec_nodes;
                tmp_j = j;
            }
        }

        // On ajoute les points dans la liste
        for (Node *n: tmp) {
            newPath->push_back(n);
        }

        // On met à jour le point courant
        currentNode = (*vecPath)[tmp_j];
        i = tmp_j;
    }
    return newPath;
}

std::vector<Node *> *AStar::lissage(std::vector<Node *> *vecPath) const {
    auto *newPath = new std::vector<Node *>();
    newPath->reserve(vecPath->size());
    newPath->push_back((*vecPath)[0]);

    Node *currentNode = (*vecPath)[0];
    int next_i = 1;
    for (int i = 1; i < vecPath->size(); i++) {
        Node *nextNode = (*vecPath)[i];

        // Si on peut tracer une ligne entre les deux noeuds
        // alors on essaye sur le suivant sinon on retourne sur le précédent
        auto vec_nodes = bresenham(currentNode, nextNode);
        while (isVectorValid(vec_nodes)) {
            if (next_i + 1 < vecPath->size()) {
                // On test avec le suivant
                nextNode = (*vecPath)[next_i + 1];
                vec_nodes = bresenham(currentNode, nextNode);

                // et on garde une trace de l'indice du noeud
                next_i++;
            } else {
                for (Node *n: vec_nodes) {
                    newPath->push_back(n);
                }
                return newPath;
            }
        }

        nextNode = (*vecPath)[next_i - 1];
        vec_nodes = bresenham(currentNode, nextNode);

        // On ajoute les noeuds de la ligne droite dans la nouvelle liste
        for (Node *n: vec_nodes) {
            newPath->push_back(n);
        }
        i = next_i - 1;
        currentNode = nextNode;
    }
    return newPath;
}

std::vector<Node *> *AStar::acceleration(std::vector<Node *> *vecPath) const {
    // Une fois le chemin optimisé il faut s'attaquer à la vitesse du kart
    // Pour cela, on va essayer de trouver la plus grande vitesse possible
    // entre 2 noeuds du chemin
    std::cout << "Accélération du chemin en cours... ";
    std::cout.flush();

    auto *newPath = new std::vector<Node *>();
    Node *currentNode = (*vecPath)[0];
    newPath->push_back(currentNode);
    int acc_max = this->map->acc_max;

    for (int i = 1; i < vecPath->size(); i++) {
        for (int j = (int) vecPath->size() - 1; j >= i; j--) {
            Node *potentialNode = (*vecPath)[j];

            int speed = std::abs(currentNode->x - potentialNode->x) + std::abs(currentNode->y - potentialNode->y);

            if (speed <= acc_max) {

                // Plusieurs algorithmes pour tracer le trait afin d'être sûr de la trajectoire
                // Notamment dû au serveur Barjokart qui a un algorithme de bresenham qui est faux
                // Les implémentations étant rapides on peut se permettre de faire plusieurs tracés
                // sans un impact important sur le temps de calcul
                auto line = bresenham(currentNode, potentialNode);
                auto line1 = bresenham(potentialNode, currentNode);
                auto line2 = bresenham2(currentNode, potentialNode);
                auto line3 = bresenham2(potentialNode, currentNode);

                if (isVectorValid(line) && isVectorValid(line1) && isVectorValid(line2) && isVectorValid(line3)) {
                    newPath->push_back(potentialNode);
                    currentNode = potentialNode;
                    i = j;
                    break;
                }
            }
        }
    }
    std::cout << "Terminé." << std::endl;
    return newPath;
}

//Cette fonction va nous donner toutes les différentes trajectoires avec le resultat d'escargot
//Grace au vecteur vitesse nous pouvons savoir quand il y a un changement de trajectoire.
std::vector<Pair> *AStar::cutting(std::vector<Pair> &vectorPath) {
    int vx, vy, vx2, vy2;

    auto *vecteur2 = new std::vector<Pair>;
    for (int k = 0; k < vectorPath.size() -1; k++) {
        vx = vectorPath.at(k).first;
        vy = vectorPath.at(k).second;

        if (k < vectorPath.size()) {
            vx2 = vectorPath.at(k + 1).first;
            vy2 = vectorPath.at(k + 1).second;
        }

        int debut = k;
        int fin;
        bool d = true;
        while (d && k < vectorPath.size()) {
            //Si les vecteurs vitesses sont différents cela signifie qu'il y a un changement de trajectoire
            if (vx != vx2 or vy != vy2) {
                fin++;
                d = false;
            } else {
                fin = k + 1;
                vx = vx2;
                vy = vy2;
                k++;
                if (k + 1 < vectorPath.size()) {
                    vx2 = vectorPath.at(k + 1).first;
                    vy2 = vectorPath.at(k + 1).second;
                } else { d = false; }
            }
        }

        if (k + 1 == vectorPath.size())
            fin++;

        //Nous avons ici la position dans VecteurPath du debut de la trajectoire jusqu'a la fin
        vecteur2->emplace_back(debut,fin);
    }
    return vecteur2;
}


std::vector<Pair> *AStar::acceleration2(std::vector<Pair> &vectorPath, std::vector<Pair> &vecteur2) {
    // Calculer les accelerations sur les différentes trajectoires

    auto *v = new std::vector<Pair>();
    int vitesse = 0;
    int longueur, vitessedefin, reste, valeurreele, Vx2, Vy2, deb, fi;
    int accel = this->map->acc_max;

    for (int n = 0; n < vecteur2.size(); n++) {
        deb = vecteur2.at(n).first;
        fi = vecteur2.at(n).second;
        std::vector deniervect = std::vector<int>();
        Vx2 = 0;
        Vy2 = 0;
        int co;
        int ar;
        if (n + 1 != vecteur2.size()) {
            int fi2 = vecteur2.at(n).second;
            int deb2 = vecteur2.at(n + 1).first;
            if (fi2 != deb2)
                fi = deb2;
        }
        int somme = fi - deb;//C'est  la longueur de la trajectoire
        if (vectorPath.at(deb).first != 0 && vectorPath.at(deb).second != 0)
            this->map->acc_max = accel / 2;
        else
            this->map->acc_max = accel;

        //si la somme est plus petit ou egal à l'acceleration alors cela veut dire qu'on peut faire la trajectoire en 1 seul coup et donc on met tout la somme
        if (somme <= this->map->acc_max) {
            deniervect.push_back(somme);
            vitessedefin = somme;
        } else {
            if (vitesse != 0) { //Si la vitesse est différente de zero alors on doit reduire cette l'acceleration par la vitesse pour eviter une tête à queue pour le premier vecteur vitesse de cette trajectoire
                deniervect.push_back(this->map->acc_max - valeurreele);
                vitesse = 0;
                somme = somme - (this->map->acc_max - valeurreele);
            }
            //Si nous sommes au la dernière trajectoire donc nous pouvons accelerer.
            if (n + 1 == vecteur2.size()) {
                int total = this->map->acc_max;
                while (0 < somme) {
                    somme = somme - total;
                    deniervect.push_back(total);
                    total = total + this->map->acc_max;
                }
            } else {
                //là c'est quand nous avons une trajectoire et qu'on doit gerer l'acceleration
                int deb2 = vecteur2.at(n + 1).first;
                int somme2 = vecteur2.at(n + 1).second - vecteur2.at(n + 1).first;
                if (vectorPath.at(deb2).first != 0 and vectorPath.at(deb2).second != 0) { //Si nous sommes dans une diagonale donc nous avancons sur X et Y donc on devise l'acceleration par 2
                    if (somme2 < this->map->acc_max) {
                        vitessedefin = this->map->acc_max;
                        longueur = somme - vitessedefin;
                        reste = longueur % this->map->acc_max;
                    } else {
                        vitessedefin = somme2 / 2 + this->map->acc_max / 2;
                        longueur = somme - vitessedefin;
                        reste = longueur % this->map->acc_max;
                    }
                } else {
                    vitessedefin = this->map->acc_max;
                    longueur = somme - vitessedefin;
                    reste = longueur % this->map->acc_max;
                }

                longueur=longueur-reste;
                // Cela permet de savoir quand on accelere ou on ralentit dans une trajectoire avec l'acceleration maximale
                // les accelerations vont être mit dans un vecteur vitesses qui se nomme deniervecteur
                while (longueur != 0) {
                    if (longueur / 3 + this->map->acc_max >= vitesse + this->map->acc_max and //Ces  condtions ont pour but qu'on accelerer mais que se soit toujours possible de reduire la vitesse sans faire de tete à queue.
                        (longueur - (vitesse + this->map->acc_max) >= vitesse) or // On accelerer tant que la longueur diviser3 +acceleration map sont plus grand vitesse et que la longueur -vitesse de l'acceleration soit plus grand que la vitesse
                        (longueur - (vitesse + this->map->acc_max) == 0 and vitesse + this->map->acc_max <= vitessedefin)) {
                        vitesse = vitesse + this->map->acc_max;
                        longueur = longueur - vitesse;
                    } else if (longueur >= vitesse) { //Si la longueur est toujours plus grand que la vitesse
                        if (longueur - vitesse <= vitesse && (longueur - vitesse) - vitesse <= (this->map->acc_max)) {
                            if (longueur - vitesse == 0 and vitesse - vitessedefin > (this->map->acc_max)) {
                                vitesse = vitesse - this->map->acc_max;
                                longueur = longueur - vitesse;
                            } else if ((longueur - vitesse >= vitesse - this->map->acc_max and //Ces conditions ont pour but que garde toujours la même vitesse
                                        longueur / vitesse - accel >= 2) or
                                       (longueur - vitesse == 0 and vitesse - vitessedefin <= accel)) {
                                longueur = longueur - vitesse;
                            } else if (vitesse - this->map->acc_max != 0) { //Cette condition à pour but d'eviter une vitesse de 0 si on reduit
                                vitesse = vitesse - this->map->acc_max;
                                longueur = longueur - vitesse;
                            } else
                                longueur = longueur - vitesse;

                        } else {
                            vitesse = vitesse - this->map->acc_max;
                            longueur = longueur - vitesse;
                        }
                    } else if (longueur >= vitesse - this->map->acc_max and vitesse - this->map->acc_max > 0) { // C'est pour reduire la vitesse si la longueur est plus grand que la vitesse - acceleration max
                        vitesse = vitesse - this->map->acc_max;
                        longueur = longueur - vitesse;
                    } else {
                        vitesse = longueur;
                        longueur = 0;
                    }
                    deniervect.push_back(vitesse);
                }
                deniervect.push_back(vitessedefin);
                if (deniervect.size() >= 3) {  // C est pour permettre si on peut mettre le reste dans l'avant dernier vitesse si c'est possible sans faire de tête à queue
                    int chif1 = deniervect.at(deniervect.size() - 3);
                    if (abs((chif1 + reste) - vitesse) <= this->map->acc_max and
                        abs(vitessedefin - (chif1 + reste)) <= this->map->acc_max) {
                        deniervect.at(deniervect.size() - 2) = deniervect.at(deniervect.size() - 2) + reste;
                    } else
                        deniervect.push_back(reste);
                }
            }
        }

        if (this->map->acc_max == accel / 2) //Si nous sommes dans une diagonale alors la vitesse est le double car on touche sur X et y
            valeurreele = vitessedefin * 2;
        else
            valeurreele = vitessedefin;

        co = deb;
        //il va se servir du vecteur deniervecteur qui possède les différentes accelerations.
        for (int p : deniervect) {
            ar = co + p;
            Vx2 = 0;
            Vy2 = 0;
            // On fait une boucle sur les qui rajoute les vecteurs vitesses obtenu d'escagot de la longueur obtenu par deniervecteur.
            for (int i = co; i < ar; i++) {
                if (ar < vectorPath.size()) {
                    Vx2 += vectorPath.at(i).first;
                    Vy2 += vectorPath.at(i).second;
                } else {
                    Vx2 += vectorPath.at(vectorPath.size() - 1).first;
                    Vx2 += vectorPath.at(vectorPath.size() - 1).second;
                }
            }
            co = ar;
            // Cela donne les un vecteur vitesse avec l'utilisation de l'acceleration qu'on rajoute dans un vecteur.
            v->emplace_back(Vx2, Vy2);
        }
    }
    return v;
}
