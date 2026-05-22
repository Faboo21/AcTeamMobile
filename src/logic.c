#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "logic.h"
#include "raycasting.h"

int calculateGasCost(int ax, int ay, int vx, int vy, char target_terrain) {
    int gas_cost = (ax * ax) + (ay * ay) + (int)(sqrt(vx * vx + vy * vy) * 1.5);
    if (target_terrain == '~') {
        gas_cost += 1;
    }
    return gas_cost;
}

void buildHeatmap(char** map, int width, int height, int** heatmap) {
    int x, y, i, nx, ny, cost;
    int queue_size = width * height;
    Point* queue;
    char** in_queue;
    int head = 0, tail = 0;
    int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    Point p;

    queue = (Point*)malloc(queue_size * sizeof(Point));
    in_queue = (char**)malloc(height * sizeof(char*));

    for (y = 0; y < height; y++) {
        in_queue[y] = (char*)calloc(width, sizeof(char));
        for (x = 0; x < width; x++) {
            heatmap[y][x] = INF;
        }
    }

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            if (map[y][x] == '=') {
                heatmap[y][x] = 0;
                queue[tail % queue_size].x = x;
                queue[tail % queue_size].y = y;
                tail++;
                in_queue[y][x] = 1;
            }
        }
    }

    while (head < tail) {
        p = queue[head % queue_size];
        head++;
        in_queue[p.y][p.x] = 0;

        for (i = 0; i < 8; i++) {
            nx = p.x + dx[i];
            ny = p.y + dy[i];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height || map[ny][nx] == '.') {
                continue;
            }

            cost = (map[ny][nx] == '~') ? 500 : 100;

            if (heatmap[p.y][p.x] + cost < heatmap[ny][nx]) {
                heatmap[ny][nx] = heatmap[p.y][p.x] + cost;
                
                if (in_queue[ny][nx] == 0) {
                    queue[tail % queue_size].x = nx;
                    queue[tail % queue_size].y = ny;
                    tail++;
                    in_queue[ny][nx] = 1;
                }
            }
        }
    }

    free(queue);
    for (y = 0; y < height; y++) {
        free(in_queue[y]);
    }
    free(in_queue);
}

int getValidMoves(CarState current, char** map, int width, int height, 
                  int p2_x, int p2_y, int p3_x, int p3_y, 
                  int try_boost, Action* validActions, CarState* validStates) {
    int count = 0;
    int min_a = try_boost ? -2 : -1;
    int max_a = try_boost ? 2 : 1;
    int ax, ay, nvx, nvy, nx, ny, speed_sq, gas_cost, next_gas, collision;
    char target_terrain;
    InfoLine line;
    Pos2Dint p;

    for (ax = min_a; ax <= max_a; ax++) {
        for (ay = min_a; ay <= max_a; ay++) {
            if (try_boost && ax >= -1 && ax <= 1 && ay >= -1 && ay <= 1) continue; 

            nvx = current.vx + ax;
            nvy = current.vy + ay;
            nx = current.x + nvx;
            ny = current.y + nvy;

            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            speed_sq = nvx * nvx + nvy * nvy;
            target_terrain = map[ny][nx];
            
            if ((map[current.y][current.x] == '~' || target_terrain == '~') && speed_sq > 1) continue;
            
            if (speed_sq > 25) continue;

            gas_cost = calculateGasCost(ax, ay, current.vx, current.vy, target_terrain);
            next_gas = current.gas - gas_cost;
            if (next_gas <= 0) continue;

            initLine(current.x, current.y, nx, ny, &line);
            
            collision = 0;
            while (nextPoint(&line, &p, 1) > 0) {
                if (p.x < 0 || p.x >= width || p.y < 0 || p.y >= height) { collision = 1; break; }
                if (map[p.y][p.x] == '.') { collision = 1; break; }
                if ((p.x == p2_x && p.y == p2_y) || (p.x == p3_x && p.y == p3_y)) { collision = 1; break; }
                if (map[p.y][p.x] == '~' && speed_sq > 1) { collision = 1; break; }
            }

            if (collision) continue;

            validActions[count].ax = ax;
            validActions[count].ay = ay;

            validStates[count].x = nx;
            validStates[count].y = ny;
            validStates[count].vx = nvx;
            validStates[count].vy = nvy;
            validStates[count].gas = next_gas;
            validStates[count].boosts = try_boost ? current.boosts - 1 : current.boosts;
            count++;
        }
    }
    return count;
}

int compareStates(const void* a, const void* b) {
    return ((CarState*)a)->score - ((CarState*)b)->score;
}

Action getBestAction(CarState current, char** map, int width, int height, int** heatmap, int p2x, int p2y, int p3x, int p3y) {
    CarState current_beam[BEAM_WIDTH];
    int current_beam_size = 1;
    CarState next_beam[BEAM_WIDTH * 25]; 
    int depth;
    int next_beam_size = 0;
    int i, m, k, use_boost, num_moves, normal_moves;
    int gas_weight;
    int min_gas_needed;
    int extra_gas;
    int safe_gas_needed;
    Action actions[25];
    CarState next_states[25];
    Action normal_actions[9];
    CarState normal_states[9];
    CarState candidate;
    Action emergency_brake;
    
    CarState best_ever;
    int best_ever_heatmap;
    int cand_hm;
    int buffer;
    
    clock_t start_time;
    double elapsed_time;

    start_time = clock();

    current_beam[0] = current;
    current_beam[0].score = heatmap[current.y][current.x];
    
    best_ever = current;
    best_ever_heatmap = heatmap[current.y][current.x];

    for (depth = 0; depth < MAX_DEPTH; depth++) {
        elapsed_time = ((double)(clock() - start_time)) / CLOCKS_PER_SEC;
        if (elapsed_time > MAX_THINKING_TIME) {
            break; 
        }

        next_beam_size = 0;

        for (i = 0; i < current_beam_size; i++) {
            use_boost = (current_beam[i].boosts > 0) ? 1 : 0;
            num_moves = getValidMoves(current_beam[i], map, width, height, p2x, p2y, p3x, p3y, use_boost, actions, next_states);

            if (use_boost) {
                normal_moves = getValidMoves(current_beam[i], map, width, height, p2x, p2y, p3x, p3y, 0, normal_actions, normal_states);
                
                for(k = 0; k < normal_moves; k++) {
                    actions[num_moves] = normal_actions[k];
                    next_states[num_moves] = normal_states[k];
                    num_moves++;
                }
            }

            for (m = 0; m < num_moves; m++) {
                candidate = next_states[m];
                
                if (depth == 0) {
                    candidate.first_action = actions[m];
                } else {
                    candidate.first_action = current_beam[i].first_action;
                }

                min_gas_needed = heatmap[candidate.y][candidate.x] / 100;
                safe_gas_needed = min_gas_needed + (min_gas_needed / 4);

                if (candidate.gas < min_gas_needed) {
                    candidate.score = 500000 + heatmap[candidate.y][candidate.x];
                } else {
                    if (candidate.gas >= safe_gas_needed * 2) {
                        gas_weight = 1;
                    } else if (candidate.gas < safe_gas_needed) {
                        gas_weight = 150; 
                    } else {
                        buffer = safe_gas_needed; 
                        if (buffer == 0) buffer = 1; 
                        
                        extra_gas = candidate.gas - safe_gas_needed;
                        if (extra_gas < 0) extra_gas = 0;
                        
                        gas_weight = 50 - ((extra_gas * 49) / buffer);
                        
                        if (gas_weight < 1) gas_weight = 1;
                        if (gas_weight > 50) gas_weight = 50;
                    }
                    candidate.score = heatmap[candidate.y][candidate.x] + ((current.gas - candidate.gas) * gas_weight);
                }
                
                cand_hm = heatmap[candidate.y][candidate.x];
                if (cand_hm < best_ever_heatmap || (cand_hm == best_ever_heatmap && candidate.score < best_ever.score)) {
                    best_ever = candidate;
                    best_ever_heatmap = cand_hm;
                }
                
                next_beam[next_beam_size++] = candidate;
            }
        }

        if (next_beam_size == 0) break;

        qsort(next_beam, next_beam_size, sizeof(CarState), compareStates);

        current_beam_size = (next_beam_size < BEAM_WIDTH) ? next_beam_size : BEAM_WIDTH;
        for (i = 0; i < current_beam_size; i++) {
            current_beam[i] = next_beam[i];
        }
        
        if (heatmap[current_beam[0].y][current_beam[0].x] == 0) break;
    }

    if (current_beam_size == 0 || (current_beam_size == 1 && next_beam_size == 0 && depth == 0)) {
        emergency_brake.ax = 0;
        emergency_brake.ay = 0;
        if (current.vx > 0) emergency_brake.ax = -1;
        if (current.vx < 0) emergency_brake.ax = 1;
        if (current.vy > 0) emergency_brake.ay = -1;
        if (current.vy < 0) emergency_brake.ay = 1;
        return emergency_brake;
    }

    return best_ever.first_action;
}
