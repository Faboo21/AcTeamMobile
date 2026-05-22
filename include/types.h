#ifndef TYPES_H
#define TYPES_H

#define MAX_LINE_LENGTH 1024
#define BOOSTS_AT_START 5
#define INF 999999
#define BEAM_WIDTH 300
#define MAX_DEPTH 300
#define MAX_THINKING_TIME 0.50

typedef struct { int x, y; } Pos2Dint;

typedef struct {
    Pos2Dint start;
    Pos2Dint end;
    struct { float x; float y; } currentPosition;
    int len;
    int pos;
    struct { float x; float y; } delta;
} InfoLine;

typedef struct {
    int ax;
    int ay;
} Action;

typedef struct {
    int x;
    int y;
    int vx;
    int vy;
    int gas;
    int boosts;
    int score;
    Action first_action;
} CarState;

typedef struct { int x, y; } Point;

#endif
