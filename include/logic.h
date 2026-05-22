#ifndef LOGIC_H
#define LOGIC_H

#include "types.h"

int calculateGasCost(int ax, int ay, int vx, int vy, char target_terrain);
void buildHeatmap(char** map, int width, int height, int** heatmap);
int getValidMoves(CarState current, char** map, int width, int height, int p2_x, int p2_y, int p3_x, int p3_y, int try_boost, Action* validActions, CarState* validStates);
Action getBestAction(CarState current, char** map, int width, int height, int** heatmap, int p2x, int p2y, int p3x, int p3y);

#endif
