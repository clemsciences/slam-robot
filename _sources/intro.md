# Introduction

## Historique
Ce document est le résultat d'une longue pérégrination autour de la robotique, des probabilités et de l'informatique.
Ca a commencé à Télécom SudParis[^tsp] dans le club de robotique INTech[^intech]. Ma première année 2013-2014 m'a permis de découvrir le travail en équipe pendant un an autour d'un robot qui devait accomplir plusieurs actions dans des matchs de 90 secondes en gagnant plus de point que le robot adverse à la Coupe de France de robotique[^cdr].


J'ai participé en 2019 à nouveau à la Coupe de France de robotique avec des anciens du club INTech. J'ai alors utilisé
Rédaction alors d'un document nommé [Les différents filtres de Kalman](https://api.clementbesnier.eu/projets/filtrage_statistique/kalman.pdf) qui résume 

## Présentation sommaire du plan

L'ouvrage est fait pour apprendre les concepts de localisation et de cartographie d'un robot. Pour ce faire, on présentera d'abord à chaque fois le paradigme mathématique, puis viendra la représentation choisie en informatique par le langage utilisé, Python en l'occurrence, pour finir avec une visualisation interactive grâce à l'interactivité des *notebooks Jupyter*. 

La robotique, telle que nous l'abordons, commence par des [rappels mathématiques sur la géométrie](1_geometry.ipynb) qui serviront à positionner l'environnment et soi-même. 
Les notions de distance, d'angle et d'alignement seront utilisées. 
Un des aspects essentiels de la robotique est l'interaction d'un agent avec son [environnement](2_environment.ipynb). Nous parlerons donc de l'environnement. 
La description sera d'abord simple puis simplifié. 
Le processus de complexification sera mené pas à pas.
Alors, nous pouvons jouer avec le [robot](3_robot_1.ipynb) ! Il ne pourra pour le moment qu'avancer tout droit et tourner.
On ajoute ensuite des yeux sous la forme d'un [LiDAR](4_lidar.ipynb). C'est un LiDAR 2D circulaire.
Jusque là, nous faisions comme si le monde était clair comme de l'eau de roche. Les actions et les mesures ont dorénavant un côté génant : de l'[incertitude](5_uncertainty.ipynb) s'y rajoute dans la réalité.
Le [filtrage](6_filtering.ipynb) est un procédé pour se sortir 



## Le robot

On a un robot qui peut faire deux choses : 
- avancer tout droit
- tourner autour de lui-même.

Au fur et à mesure, le robot acquerrera de nouvelles capacités, ainsi que de nouveaux capteurs et actionneurs.


:::{note}
C'est une note.
:::

SLAM

[^tsp]: [Télécom SudParis](https://www.telecom-sudparis.eu/)
[^intech]: [Présentation d'INTech](https://gate.wp.telecom-sudparis.eu/projet/intech-2024-tech-the-turf/)
[^cdr]: [Site officiel de la Coupe de France de robotique](https://www.coupederobotique.fr/)
