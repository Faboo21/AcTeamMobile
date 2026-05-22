#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "logic.h"
#include "raycasting.h"

int main() {
    int width, height, gasLevel;
    char line_buffer[MAX_LINE_LENGTH];
    char** map;
    int row;
    int** heatmap;
    CarState my_car;
    int round = 0;
    int myX, myY, secondX, secondY, thirdX, thirdY;
    Action best_action;
    int target_nx, target_ny;
    char target_terrain;
    
    int prev_x = -1;
    int prev_y = -1;

    fgets(line_buffer, MAX_LINE_LENGTH, stdin);
    sscanf(line_buffer, "%d %d %d", &width, &height, &gasLevel);

    map = (char**)malloc(height * sizeof(char*));
    for (row = 0; row < height; ++row) {
        map[row] = (char*)malloc(MAX_LINE_LENGTH * sizeof(char));
        fgets(map[row], MAX_LINE_LENGTH, stdin);
        fputs(map[row], stderr);
    }
    fflush(stderr);

    heatmap = (int**)malloc(height * sizeof(int*));
    for (row = 0; row < height; ++row) {
        heatmap[row] = (int*)malloc(width * sizeof(int));
    }
    buildHeatmap(map, width, height, heatmap);
    
    my_car.vx = 0;
    my_car.vy = 0;
    my_car.gas = gasLevel;
    my_car.boosts = BOOSTS_AT_START;

    while (!feof(stdin)) {
        round++;
        
        fgets(line_buffer, MAX_LINE_LENGTH, stdin);
        if (strlen(line_buffer) < 5) break; 
        
        sscanf(line_buffer, "%d %d %d %d %d %d", &myX, &myY, &secondX, &secondY, &thirdX, &thirdY);
        
        if (round > 1) {
            my_car.vx = myX - prev_x;
            my_car.vy = myY - prev_y;
        } else {
            my_car.vx = 0;
            my_car.vy = 0;
        }
        
        my_car.x = myX;
        my_car.y = myY;

        prev_x = myX;
        prev_y = myY;

        best_action = getBestAction(my_car, map, width, height, heatmap, secondX, secondY, thirdX, thirdY);

        target_nx = my_car.x + my_car.vx + best_action.ax;
        target_ny = my_car.y + my_car.vy + best_action.ay;
        target_terrain = '#';
        
        if (target_nx >= 0 && target_nx < width && target_ny >= 0 && target_ny < height) {
            target_terrain = map[target_ny][target_nx];
        }
        
        my_car.gas -= calculateGasCost(best_action.ax, best_action.ay, my_car.vx, my_car.vy, target_terrain);
        my_car.vx += best_action.ax;
        my_car.vy += best_action.ay;
        
        if (abs(best_action.ax) == 2 || abs(best_action.ay) == 2) {
            my_car.boosts--;
        }

        fprintf(stdout, "%d %d\n", best_action.ax, best_action.ay);
        fflush(stdout);
    }

    for (row = 0; row < height; ++row) {
        free(map[row]);
        free(heatmap[row]);
    }
    free(map);
    free(heatmap);

    return EXIT_SUCCESS;
}
