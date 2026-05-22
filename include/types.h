/**
 * @file types.h
 * @brief Définitions des constantes et structures de données globales.
 * @author AcTeamMobile
 */
#ifndef TYPES_H
#define TYPES_H

/** @brief Longueur maximale d'une ligne lue sur l'entrée standard. */
#define MAX_LINE_LENGTH 1024
/** @brief Nombre de boosts disponibles au début de la course. */
#define BOOSTS_AT_START 5
/** @brief Valeur représentant l'infini pour l'initialisation de la Heatmap. */
#define INF 999999
/** @brief Largeur du faisceau pour l'algorithme Beam Search. */
#define BEAM_WIDTH 300
/** @brief Profondeur maximale de recherche pour l'algorithme Beam Search. */
#define MAX_DEPTH 300
/** @brief Temps de réflexion maximum autorisé par tour en secondes. */
#define MAX_THINKING_TIME 0.50

/**
 * @brief Structure représentant une position en coordonnées entières 2D.
 */
typedef struct { int x, y; } Pos2Dint;

/**
 * @brief Structure utilisée pour le calcul de trajectoire (algorithme de tracé de ligne).
 */
typedef struct {
    Pos2Dint start;
    Pos2Dint end;
    struct { float x; float y; } currentPosition;
    int len;
    int pos;
    struct { float x; float y; } delta;
} InfoLine;

/**
 * @brief Structure représentant une action (accélération) du véhicule.
 */
typedef struct {
    int ax;
    int ay;
} Action;

/**
 * @brief Structure représentant l'état du véhicule à un instant t.
 */
typedef struct {
    int x;      /**< Position X */
    int y;      /**< Position Y */
    int vx;     /**< Vitesse X */
    int vy;     /**< Vitesse Y */
    int gas;    /**< Carburant restant */
    int boosts; /**< Nombre de boosts restants */
    int score;  /**< Score heuristique évalué de cet état */
    Action first_action; /**< L'action initiale qui a mené à cet état */
} CarState;

/**
 * @brief Structure utilitaire pour une coordonnée 2D (utilisée dans la file BFS).
 */
typedef struct { int x, y; } Point;

#endif
