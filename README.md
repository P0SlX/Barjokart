# Barjokart
Projet de fin d'année de L3. Le but étant de partir d'un PNG combiné à un fichier TOML et d'arriver le plus rapidement possible aux points d'arrivés.  
Nous avons choisi d'utiliser l'algorithme A* afin d'avoir un chemin rapidement et quasiment optimal dès le début. Plusieurs algorithmes (encore à perfectionner) sont utilisés pour "lisser" le chemin et augmenter la vitesse du Kart, car le but du projet est de combiner la plus haute vélocité et la plus courte distance.
## Compiler
Dans la racine du projet
```
mkdir build
cd build
cmake ..
make
```

## Lancement
`./barjokart`  

Il faut ensuite choisir entre les 2 algorithmes d'accélération.  
Les résultats sont dans le dossier `output`

