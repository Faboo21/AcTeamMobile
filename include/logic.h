/**
 * @file logic.h
 * @brief Logique métier et intelligence artificielle (Heatmap, validité des mouvements, Beam Search).
 */
#ifndef LOGIC_H
#define LOGIC_H

#include "types.h"

/**
 * @brief Calcule le coût en carburant d'une action spécifique.
 * @param ax Accélération en X.
 * @param ay Accélération en Y.
 * @param vx Vitesse actuelle en X.
 * @param vy Vitesse actuelle en Y.
 * @param target_terrain Type de terrain de la case d'arrivée.
 * @return Le coût en unités de carburant.
 */
int calculateGasCost(int ax, int ay, int vx, int vy, char target_terrain);

/**
 * @brief Génère une Heatmap (carte des distances) par parcours en largeur (BFS).
 * @param map Le circuit sous forme de grille de caractères.
 * @param width Largeur de la carte.
 * @param height Hauteur de la carte.
 * @param heatmap Tableau 2D d'entiers à remplir avec les coûts de distance.
 */
void buildHeatmap(char** map, int width, int height, int** heatmap);

/**
 * @brief Détermine tous les mouvements légaux à partir d'un état donné.
 * @param current État actuel du véhicule.
 * @param map La grille du circuit.
 * @param width Largeur de la carte.
 * @param height Hauteur de la carte.
 * @param p2_x Position X de l'adversaire 1.
 * @param p2_y Position Y de l'adversaire 1.
 * @param p3_x Position X de l'adversaire 2.
 * @param p3_y Position Y de l'adversaire 2.
 * @param try_boost 1 si on tente de générer des actions de boost, 0 sinon.
 * @param validActions Tableau de sortie pour stocker les actions valides.
 * @param validStates Tableau de sortie pour stocker les états résultants valides.
 * @return Le nombre de mouvements valides trouvés.
 */
int getValidMoves(CarState current, char** map, int width, int height, int p2_x, int p2_y, int p3_x, int p3_y, int try_boost, Action* validActions, CarState* validStates);

/**
 * @brief Recherche la meilleure action à jouer selon l'algorithme Beam Search.
 * @param current État actuel de notre véhicule.
 * @param map La grille du circuit.
 * @param width Largeur de la carte.
 * @param height Hauteur de la carte.
 * @param heatmap La Heatmap des distances à la ligne d'arrivée.
 * @param p2x Position X de l'adversaire 1.
 * @param p2y Position Y de l'adversaire 1.
 * @param p3x Position X de l'adversaire 2.
 * @param p3y Position Y de l'adversaire 2.
 * @return L'action optimale à effectuer (accélération X et Y).
 */
Action getBestAction(CarState current, char** map, int width, int height, int** heatmap, int p2x, int p2y, int p3x, int p3y);

#endif
