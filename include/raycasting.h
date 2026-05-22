/**
 * @file raycasting.h
 * @brief Déclarations des fonctions de calcul de trajectoire par lancer de rayon (raycasting).
 */
#ifndef RAYCASTING_H
#define RAYCASTING_H

#include "types.h"

/**
 * @brief Initialise une ligne pour parcourir les points intermédiaires d'un déplacement.
 * @param x1 Coordonnée X de départ.
 * @param y1 Coordonnée Y de départ.
 * @param x2 Coordonnée X d'arrivée.
 * @param y2 Coordonnée Y d'arrivée.
 * @param infoLine Pointeur vers la structure InfoLine à initialiser.
 */
void initLine(int x1, int y1, int x2, int y2, InfoLine * infoLine);

/**
 * @brief Récupère le prochain point intermédiaire sur la ligne tracée.
 * @param infoLine Pointeur vers la structure InfoLine contenant l'état du tracé.
 * @param point Pointeur vers la structure Pos2Dint où stocker les coordonnées trouvées.
 * @param direction Direction de progression (habituellement 1).
 * @return 1 si un point a été trouvé, -1 si la fin de la ligne est atteinte.
 */
int nextPoint(InfoLine * infoLine, Pos2Dint * point, int direction);

#endif
