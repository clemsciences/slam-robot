# SLAM for a robot


## Make the book
### Generate the book

```bash
jupyter-book build src/course
```
### Update the book
```bash
ghp-import -n -p -f src/course/_build/html
```

```bash
pip install -e .
rebuild-book
```


#%% md
# Le lidar pour voir

Normalement, on devrait écrire LiDAR pour (**Li**ght **D**etection **A**nd **R**anging), mais lidar est plus facile à lire. De plus, on suit le destin qu'a pris 
laser (**L**ight **A**mplification by **S**timulated **E**mission of **R**adiation). 

Un lidar à balayage rotatif (ou *scanning lidar*) est un capteur actif qui permet d'obtenir une cartographie 2D de l'environnement immédiat du robot en mesurant la distance des obstacles sur 360°.

## Fonctionnement d'un lidar à balayage

Le but d'un lidar est de mesurer la distance des obstacles autour de celui-ci.

Le lidar qui nous intéresse est constitué d'un laser, d'un récepteur optique et d'une partie mécanique capable de tourner autour d'un axe vertical.

Le lidar tourne autour de son axe de rotation avec le laser en fonctionnement et le récepteur qui mesure le signal retourné au cours du temps.

### Principes de mesure de distance

Il existe principalement deux techniques utilisées par les lidars 2D pour mesurer la distance $d$ :

1. **Le Temps de Vol (Time of Flight - ToF)** :
   Le lidar émet une impulsion lumineuse et mesure le temps $\Delta t$ qu'elle met pour revenir après s'être réfléchie sur un obstacle. La distance est alors donnée par :
   $$d = \frac{c \cdot \Delta t}{2}$$
   où $c$ est la vitesse de la lumière ($c \approx 3 \cdot 10^8$ m/s). Cette méthode est précise pour les longues distances.

2. **La Triangulation optique** :
   Le laser émet un faisceau continu. L'image du point d'impact sur l'obstacle est projetée via une lentille sur un capteur linéaire (CCD ou CMOS). La position du point sur le capteur dépend de la distance de l'obstacle. C'est une technique courante sur les lidars bas coût (comme le RPLidar A1).

### Composants principaux

* **Émetteur** : Une diode laser émettant généralement dans l'infrarouge (souvent autour de 785 nm ou 905 nm pour des raisons de sécurité oculaire).
* **Récepteur** : Un photodétecteur (photodiode ou APD) sensible à la longueur d'onde du laser.
* **Système de balayage** : Un moteur fait tourner l'ensemble (émetteur/récepteur) ou un miroir pour balayer l'espace sur 360°.
* **Encodeur optique** : Permet de connaître précisément l'angle $\theta$ auquel la mesure de distance est effectuée.

### Caractéristiques techniques

* **Portée (Range)** : Distance minimale et maximale mesurable (ex: 0.15m à 12m).
* **Résolution angulaire** : L'écart entre deux mesures successives (ex: 1° ou 0.5°).
* **Fréquence de balayage** : Vitesse de rotation (ex: 5 Hz à 15 Hz).
* **Fréquence d'échantillonnage** : Nombre de mesures par seconde (ex: 2000 à 8000 points/s).

## Distinguer les obstacles et les balises