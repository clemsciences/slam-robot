

Tout commence à INTech.
Tout a commencé avec le [projet Hermes](https://github.com/hermes-project) qui devait servir à concevoir un robot.
Il y avait entre autres un LiDAR qui avait son propre [projet](https://github.com/hermes-project/lidar).
J'ai repris le [projet](https://github.com/clemsciences/lidar-gobgob) (sans que ça soit un fork).
J'ai conçu un [projet parallèle](https://github.com/gobgob/lidar-processor) qui s'inspire en partie du projet précédent pour la coupe de France de robotique 2019.

Pour le LiDAR, il fallait coder le [pilote du RP LiDAR 3](https://github.com/gobgob/rplidar_a3) qui s'inpire du [projet fait à INTech pour le RP LiDAr 2](https://github.com/Club-INTech/rplidar_a2).

Après la coupe de France de robotique 2019, j'ai voulu expliquer l'essence de ce qui a été fait pour traiter les données du LiDAR dans ce [projet](https://github.com/clemsciences/lidar-notebooks).
J'ai aussi voulu un temps avoir un [outil de visualisation des mesures LiDAR](https://github.com/clemsciences/lidar-server-flask-vue) en temps-réel. Mais ça n'a pas été loin.

L'objectif était que le robot puisse se localiser sur le terrain en temps réel à l'aide de balise. Cela marchait en statique mais pas en mouvement.
SLAM (Self Location And Mapping) devenait intéressant mais la correspondance des points d'une image statique à l'autre de nuages de points est une tâche bien complexe. 
ICP (Iterative Closest Point) est la méthode privilégiée pour faire correspondre des nuages de points.
Mais cela a échoué. D'où ce projet.