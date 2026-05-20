#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define MAX_LINE_LENGTH 1024
#define MAX_WIDTH 250
#define MAX_HEIGHT 250
#define INF 999999
#define BOOSTS_AT_START 5

#define ENABLE_DEBUG_LOG 1

void debug_log(const char *format, ...) {
#if ENABLE_DEBUG_LOG
    va_list args;
    va_start(args, format);
    fprintf(stderr, "[DEBUG] ");
    vfprintf(stderr, format, args);
    fflush(stderr);
    va_end(args);
#endif
}

char map[MAX_HEIGHT][MAX_WIDTH];
int dist[MAX_HEIGHT][MAX_WIDTH];
int width, height;

/* ========================================================================= */
/* 0. MODULE FOLLOW_LINE EMBARQUE                                            */
/* ========================================================================= */
typedef struct { int x; int y; } Pos2Dint;
typedef struct { float x; float y; } Pos2Dfloat;

typedef struct {
    Pos2Dint start;
    Pos2Dint end;
    Pos2Dfloat currentPosition;
    Pos2Dfloat delta;
    int pos;
    int len;
} InfoLine;

void initLine(int x1, int y1, int x2, int y2, InfoLine * infoLine) {
    int adxi, adyi, dxi, dyi;
    infoLine->start.x = x1;
    infoLine->start.y = y1;
    infoLine->currentPosition.x = x1 + 0.5;
    infoLine->currentPosition.y = y1 + 0.5;
    infoLine->end.x = x2;
    infoLine->end.y = y2;

    adxi = dxi = x2 - x1;
    adyi = dyi = y2 - y1;
    if (adxi < 0) adxi = -dxi;
    if (adyi < 0) adyi = -dyi;

    infoLine->pos = 0;
    infoLine->len = adxi;
    if (adyi > adxi) infoLine->len = adyi;

    infoLine->delta.x = ((float)dxi) / infoLine->len;
    infoLine->delta.y = ((float)dyi) / infoLine->len;
}

int nextPoint(InfoLine * infoLine, Pos2Dint * point, int direction) {
    if (direction > 0) {
        if (infoLine->pos == infoLine->len) {
            point->x = infoLine->end.x;
            point->y = infoLine->end.y;
            return -1;
        }
        infoLine->currentPosition.x += infoLine->delta.x;
        infoLine->currentPosition.y += infoLine->delta.y;
        point->x = ((int)infoLine->currentPosition.x);
        point->y = ((int)infoLine->currentPosition.y);
        infoLine->pos++;
        return 1;
    }
    if (direction < 0) {
        if (infoLine->pos == 0) {
            point->x = infoLine->start.x;
            point->y = infoLine->start.y;
            return -1;
        }
        infoLine->currentPosition.x -= infoLine->delta.x;
        infoLine->currentPosition.y -= infoLine->delta.y;
        point->x = ((int)infoLine->currentPosition.x);
        point->y = ((int)infoLine->currentPosition.y);
        infoLine->pos--;
        return 1;
    }
    return 1;
}

/* ========================================================================= */
/* 1. BFS PRE-COMPUTATION                                                    */
/* ========================================================================= */
typedef struct { int x, y; } Point;
Point queue[MAX_WIDTH * MAX_HEIGHT];
int q_head = 0, q_tail = 0;

void push(int x, int y) { queue[q_tail].x = x; queue[q_tail].y = y; q_tail++; }
Point pop() { return queue[q_head++]; }
int is_empty() { return q_head == q_tail; }

void precompute_distances() {
    int x, y, i;
    int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
    int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            dist[y][x] = INF;
            if (map[y][x] == '=') { dist[y][x] = 0; push(x, y); }
        }
    }

    while (!is_empty()) {
        Point p = pop();
        int current_dist = dist[p.y][p.x];
        for (i = 0; i < 8; i++) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            if (nx >= 0 && nx < width && ny >= 0 && ny < height && map[ny][nx] != '.') {
                int cost = (map[ny][nx] == '~') ? 5 : 1;
                if (dist[ny][nx] > current_dist + cost) {
                    dist[ny][nx] = current_dist + cost;
                    push(nx, ny);
                }
            }
        }
    }
}

/* ========================================================================= */
/* 2. OUTILS DE CONSOMMATION ET DE COLLISION                                 */
/* ========================================================================= */
int gasConsumption(int accX, int accY, int speedX, int speedY, int inSand) {
    int gas = accX * accX + accY * accY;
    gas += (int)(sqrt(speedX * speedX + speedY * speedY) * 1.5); /* 1.5 == 3.0 / 2.0 */
    if (inSand) gas += 1;
    return -gas;
}

int is_valid_trajectory(int x1, int y1, int x2, int y2, int speed_sq) {
    InfoLine vline;
    Pos2Dint p;

    if (x2 < 0 || x2 >= width || y2 < 0 || y2 >= height) return 0;

    initLine(x1, y1, x2, y2, &vline);
    while (nextPoint(&vline, &p, +1) > 0) {
        if (p.x == x1 && p.y == y1) continue;

        if (p.x < 0 || p.x >= width || p.y < 0 || p.y >= height) return 0;
        if (map[p.y][p.x] == '.') return 0;
        if (map[p.y][p.x] == '~' && speed_sq > 1) return 0;
    }
    return 1;
}

int is_safe(int x, int y, int vx, int vy) {
    int cx = x, cy = y;
    int nx, ny, speed_sq;

    if (cx >= 0 && cx < width && cy >= 0 && cy < height && map[cy][cx] == '=') return 1;

    while (vx != 0 || vy != 0) {
        if (vx > 0) vx--; else if (vx < 0) vx++;
        if (vy > 0) vy--; else if (vy < 0) vy++;

        nx = cx + vx;
        ny = cy + vy;
        speed_sq = vx*vx + vy*vy;

        if (!is_valid_trajectory(cx, cy, nx, ny, speed_sq)) return 0;
        if (nx >= 0 && nx < width && ny >= 0 && ny < height && map[ny][nx] == '=') return 1;

        cx = nx;
        cy = ny;
    }
    return 1;
}

/* ========================================================================= */
/* 3. A* CINEMATIQUE DYNAMIQUE                                               */
/* ========================================================================= */
#define MAX_NODES 30000

typedef struct {
    int x, y, vx, vy;
    int ax, ay;
    int g, f;
    int fuel;
    int boosts;
    int parent_id;
} AStarNode;

AStarNode tree[MAX_NODES];
int tree_size = 0;
int heap[MAX_NODES];
int heap_size = 0;

int visited_id[MAX_HEIGHT][MAX_WIDTH][11][11];
int best_cost[MAX_HEIGHT][MAX_WIDTH][11][11];
int current_search_id = 0;

void heap_push(int node_id) {
    int i = heap_size++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (tree[heap[p]].f <= tree[node_id].f) break;
        heap[i] = heap[p];
        i = p;
    }
    heap[i] = node_id;
}

int heap_pop() {
    int ret = heap[0];
    int last = heap[--heap_size];
    int i = 0, left, right, min_child;

    if (heap_size == 0) return ret;

    while (i * 2 + 1 < heap_size) {
        left = i * 2 + 1;
        right = i * 2 + 2;
        min_child = left;
        if (right < heap_size && tree[heap[right]].f < tree[heap[left]].f) min_child = right;
        if (tree[last].f <= tree[heap[min_child]].f) break;
        heap[i] = heap[min_child];
        i = min_child;
    }
    heap[i] = last;
    return ret;
}

void get_best_move(int start_x, int start_y, int start_vx, int start_vy,
                   int *out_ax, int *out_ay,
                   int opp1_x, int opp1_y, int opp2_x, int opp2_y, int round, int gasLevel, int current_boosts) {
    int best_node_id = 0;
    int min_h_seen = INF;
    int curr_id, ax, ay;
    float fuel_ratio;

    int max_speed_sq, can_boost, gas_weight, time_weight, boost_cost;
    int max_acc;

    if (dist[start_y][start_x] == 0) {
        *out_ax = 0; *out_ay = 0;
        return;
    }

    /* L'IA DECIDE DE SA STRATEGIE EN FONCTION DU RATIO ESSENCE/DISTANCE */
    fuel_ratio = (float)gasLevel / (dist[start_y][start_x] + 1);

    if (fuel_ratio < 1.2) {
        max_speed_sq = 4;      /* Vitesse limitee a 2 */
        can_boost = 0;         /* Interdiction d'utiliser les boosts (coutent trop cher) */
        gas_weight = 50000;    /* Economiser l'essence est une question de vie ou de mort */
        time_weight = 1000;    /* Le chrono n'a plus d'importance */
        debug_log("Mode: SURVIE EXTREME (Ratio: %.2f) - Vitesse max 2\n", fuel_ratio);
    } else if (fuel_ratio < 1.5) {
        max_speed_sq = 9;      /* Vitesse limitee a 3 */
        can_boost = 0;
        gas_weight = 10000;
        time_weight = 2000;
        debug_log("Mode: SURVIE (Ratio: %.2f) - Vitesse max 3\n", fuel_ratio);
    } else if (fuel_ratio < 2.0) {
        max_speed_sq = 16;     /* Vitesse limitee a 4 */
        can_boost = 0;
        gas_weight = 5000;
        time_weight = 5000;
        debug_log("Mode: ECO (Ratio: %.2f) - Vitesse max 4\n", fuel_ratio);
    } else {
        max_speed_sq = 25;     /* Vitesse max a 5 */
        can_boost = 1;         /* Autorisation des Boosts ! */
        gas_weight = 10;       /* On se fiche un peu de l'essence */
        time_weight = 10000;   /* On veut le meilleur chrono possible */
        debug_log("Mode: COURSE (Ratio: %.2f) - Plein Gaz (Boosts OK)\n", fuel_ratio);
    }

    max_acc = can_boost ? 2 : 1;
    boost_cost = time_weight / 10; /* Un boost "coute" virtuellement un petit peu pour ne pas les cramer sans raison */

    current_search_id++;
    tree_size = 0;
    heap_size = 0;

    tree[0].x = start_x; tree[0].y = start_y;
    tree[0].vx = start_vx; tree[0].vy = start_vy;
    tree[0].g = 0;
    tree[0].f = dist[start_y][start_x] * (time_weight / 5 + gas_weight);
    tree[0].fuel = gasLevel;
    tree[0].boosts = current_boosts;
    tree[0].parent_id = -1;
    tree_size++;

    heap_push(0);
    visited_id[start_y][start_x][start_vx + 5][start_vy + 5] = current_search_id;
    best_cost[start_y][start_x][start_vx + 5][start_vy + 5] = 0;

    while (heap_size > 0 && tree_size < MAX_NODES) {
        curr_id = heap_pop();

        if (dist[tree[curr_id].y][tree[curr_id].x] == 0) {
            best_node_id = curr_id;
            break;
        }

        for (ax = -max_acc; ax <= max_acc; ax++) {
            for (ay = -max_acc; ay <= max_acc; ay++) {
                int nx, ny, nvx, nvy, h, new_g, new_f;
                int speed_sq, is_goal, move_gas_cost, new_fuel, gas_spent, state_cost;
                int uses_boost, new_boosts, h_cost;

                uses_boost = (abs(ax) == 2 || abs(ay) == 2);
                if (uses_boost && tree[curr_id].boosts <= 0) continue;

                nvx = tree[curr_id].vx + ax;
                nvy = tree[curr_id].vy + ay;
                speed_sq = nvx * nvx + nvy * nvy;

                if (speed_sq > max_speed_sq) continue;

                nx = tree[curr_id].x + nvx;
                ny = tree[curr_id].y + nvy;

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                is_goal = (map[ny][nx] == '=');

                move_gas_cost = -gasConsumption(ax, ay, nvx, nvy, (map[ny][nx] == '~'));
                new_fuel = tree[curr_id].fuel - move_gas_cost;

                /* REGLE ABSOLUE : Si on tombe a 0 sans avoir franchi la ligne, on jette ce chemin */
                if (new_fuel < 0 || (new_fuel == 0 && !is_goal)) continue;

                new_g = tree[curr_id].g + 1;
                new_boosts = tree[curr_id].boosts - uses_boost;
                gas_spent = gasLevel - new_fuel;

                /* Le cout prend compte du temps ET de l'essence de maniere dynamique */
                state_cost = (new_g * time_weight) + (gas_spent * gas_weight) + (uses_boost * boost_cost);

                if (visited_id[ny][nx][nvx + 5][nvy + 5] == current_search_id) {
                    if (state_cost >= best_cost[ny][nx][nvx + 5][nvy + 5]) continue;
                }

                if (is_valid_trajectory(tree[curr_id].x, tree[curr_id].y, nx, ny, speed_sq) && (is_goal || is_safe(nx, ny, nvx, nvy))) {

                    visited_id[ny][nx][nvx + 5][nvy + 5] = current_search_id;
                    best_cost[ny][nx][nvx + 5][nvy + 5] = state_cost;

                    h = dist[ny][nx];
                    if ((nx == opp1_x && ny == opp1_y) || (nx == opp2_x && ny == opp2_y)) h += 100;

                    h_cost = h * ((time_weight / 5) + gas_weight);
                    new_f = state_cost + h_cost;

                    tree[tree_size].x = nx; tree[tree_size].y = ny;
                    tree[tree_size].vx = nvx; tree[tree_size].vy = nvy;
                    tree[tree_size].ax = ax; tree[tree_size].ay = ay;
                    tree[tree_size].g = new_g;
                    tree[tree_size].f = new_f;
                    tree[tree_size].fuel = new_fuel;
                    tree[tree_size].boosts = new_boosts;
                    tree[tree_size].parent_id = curr_id;

                    if (h < min_h_seen) {
                        min_h_seen = h;
                        best_node_id = tree_size;
                    }

                    heap_push(tree_size);
                    tree_size++;

                    if (tree_size >= MAX_NODES) break;
                }
            }
            if (tree_size >= MAX_NODES) break;
        }
    }

    if (best_node_id == 0) {
        *out_ax = (start_vx > 0) ? -1 : ((start_vx < 0) ? 1 : 0);
        *out_ay = (start_vy > 0) ? -1 : ((start_vy < 0) ? 1 : 0);
        return;
    }

    curr_id = best_node_id;
    while (tree[curr_id].parent_id != 0) {
        curr_id = tree[curr_id].parent_id;
    }

    *out_ax = tree[curr_id].ax;
    *out_ay = tree[curr_id].ay;
}

/* ========================================================================= */
/* 4. MAIN PROGRAM                                                           */
/* ========================================================================= */
int main() {
    int gasLevel, round = 0, row, col;
    int speedX = 0, speedY = 0;
    int prev_myX = -1, prev_myY = -1;
    int my_boosts = BOOSTS_AT_START;
    char line_buffer[MAX_LINE_LENGTH];

    fgets(line_buffer, MAX_LINE_LENGTH, stdin);
    sscanf(line_buffer, "%d %d %d", &width, &height, &gasLevel);

    for (row = 0; row < height; ++row) {
        do {
            fgets(line_buffer, MAX_LINE_LENGTH, stdin);
        } while (line_buffer[0] == '\n' || line_buffer[0] == '\r');

        for (col = 0; col < width; ++col) {
            map[row][col] = line_buffer[col];
        }
        map[row][width] = '\0';
    }

    precompute_distances();
    memset(visited_id, 0, sizeof(visited_id));
    memset(best_cost, 0, sizeof(best_cost));

    while (!feof(stdin)) {
        int myX, myY, secondX, secondY, thirdX, thirdY;
        int ax = 0, ay = 0, inSand;

        round++;

        while (fgets(line_buffer, MAX_LINE_LENGTH, stdin) != NULL) {
            if (line_buffer[0] == '\n' || line_buffer[0] == '\r') continue;
            if (sscanf(line_buffer, "%d %d %d %d %d %d", &myX, &myY, &secondX, &secondY, &thirdX, &thirdY) == 6) break;
        }
        if (feof(stdin)) break;

        if (prev_myX != -1) {
            speedX = myX - prev_myX;
            speedY = myY - prev_myY;
        }
        prev_myX = myX;
        prev_myY = myY;

        get_best_move(myX, myY, speedX, speedY, &ax, &ay, secondX, secondY, thirdX, thirdY, round, gasLevel, my_boosts);

        if (abs(ax) == 2 || abs(ay) == 2) {
            my_boosts--;
        }

        speedX += ax;
        speedY += ay;

        inSand = (map[myY][myX] == '~') ? 1 : 0;
        gasLevel += gasConsumption(ax, ay, speedX, speedY, inSand);

        fprintf(stdout, "%d %d\n", ax, ay);
        fflush(stdout);
    }

    return EXIT_SUCCESS;
}