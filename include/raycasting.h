#ifndef RAYCASTING_H
#define RAYCASTING_H

#include "types.h"

void initLine(int x1, int y1, int x2, int y2, InfoLine * infoLine);
int nextPoint(InfoLine * infoLine, Pos2Dint * point, int direction);

#endif
