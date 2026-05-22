#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_LINE_LENGTH 1024
#define BOOSTS_AT_START 5
#define INF 999999
#define BEAM_WIDTH 200
#define MAX_DEPTH 200

/* ========================================================================= */
/* 1. STRUCTURES DE DONNEES                                                  */
/* ========================================================================= */

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

/* ========================================================================= */
/* 2. FONCTIONS DE RAYCASTING                                                */
/* ========================================================================= */

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
    
    /* Protection division par zero si deplacement nul */
    if (infoLine->len == 0) {
        infoLine->delta.x = 0;
        infoLine->delta.y = 0;
    } else {
        infoLine->delta.x = ((float)dxi) / infoLine->len;
        infoLine->delta.y = ((float)dyi) / infoLine->len;
    }
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
    return 1;
}

/* ========================================================================= */
/* 3. LOGIQUE DU JEU & ANTICIPATION                                          */
/* ========================================================================= */

int calculateGasCost(int ax, int ay, int vx, int vy, char target_terrain) {
    /* Le * 1.5 doit se faire APRES la racine carree, comme dans le GDC ! */
    int gas_cost = (ax * ax) + (ay * ay) + (int)(sqrt(vx * vx + vy * vy) * 1.5);
    if (target_terrain == '~') {
        gas_cost += 1;
    }
    return gas_cost;
}

void buildHeatmap(char** map, int width, int height, int** heatmap) {
    /* Declaration de TOUTES les variables en haut (Norme C90) */
    int x, y, i, nx, ny, cost;
    int queue_size = width * height;
    Point* queue;
    char** in_queue; /* NOUVEAU : Tableau pour savoir si une case est déjà dans la file */
    int head = 0, tail = 0;
    int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    Point p;

    queue = (Point*)malloc(queue_size * sizeof(Point));
    in_queue = (char**)malloc(height * sizeof(char*));

    /* Initialisation de la heatmap et du tableau in_queue */
    for (y = 0; y < height; y++) {
        in_queue[y] = (char*)calloc(width, sizeof(char)); /* calloc met tout à 0 (faux) */
        for (x = 0; x < width; x++) {
            heatmap[y][x] = INF;
        }
    }

    /* Trouver la ligne d'arrivée */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            if (map[y][x] == '=') {
                heatmap[y][x] = 0;
                /* NOUVEAU : utilisation du modulo (%) pour faire une file circulaire */
                queue[tail % queue_size].x = x;
                queue[tail % queue_size].y = y;
                tail++;
                in_queue[y][x] = 1; /* La case est maintenant dans la file */
            }
        }
    }

    /* Parcours de la carte */
    while (head < tail) {
        p = queue[head % queue_size];
        head++;
        in_queue[p.y][p.x] = 0; /* NOUVEAU : La case sort de la file, on la marque à 0 */

        for (i = 0; i < 8; i++) {
            nx = p.x + dx[i];
            ny = p.y + dy[i];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height || map[ny][nx] == '.') {
                continue;
            }

            cost = (map[ny][nx] == '~') ? 500 : 100;

            if (heatmap[p.y][p.x] + cost < heatmap[ny][nx]) {
                heatmap[ny][nx] = heatmap[p.y][p.x] + cost;
                
                /* NOUVEAU : On ajoute à la file UNIQUEMENT si elle n'y est pas déjà */
                if (in_queue[ny][nx] == 0) {
                    queue[tail % queue_size].x = nx;
                    queue[tail % queue_size].y = ny;
                    tail++;
                    in_queue[ny][nx] = 1; /* On indique qu'elle est dans la file */
                }
            }
        }
    }

    /* Nettoyage de la mémoire */
    free(queue);
    for (y = 0; y < height; y++) {
        free(in_queue[y]);
    }
    free(in_queue);
}

int getValidMoves(CarState current, char** map, int width, int height, 
                  int p2_x, int p2_y, int p3_x, int p3_y, 
                  int try_boost, Action* validActions, CarState* validStates) {
    /* Declaration de TOUTES les variables en haut */
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
            
            /* --- CORRECTION SABLE 1 : Vérifier la case actuelle --- */
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
                /* --- CORRECTION SABLE 2 : Vérifier les cases traversées --- */
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
    /* --- DECLARATION DE TOUTES LES VARIABLES (NORME C90) --- */
    CarState current_beam[BEAM_WIDTH];
    int current_beam_size = 1;
    CarState next_beam[BEAM_WIDTH * 25]; 
    int depth;
    int next_beam_size = 0;
    int i, m, k, use_boost, num_moves, normal_moves;
    int gas_weight;
    int min_gas_needed;
    int extra_gas;
    Action actions[25];
    CarState next_states[25];
    Action normal_actions[9];
    CarState normal_states[9];
    CarState candidate;
    Action emergency_brake;
    
    /* Variables pour le "baroud d'honneur" et la memoire du meilleur etat */
    CarState best_ever;
    int best_ever_heatmap;
    int cand_hm;
    int buffer;
    
    /* Variables pour le chronometre */
    clock_t start_time;
    double elapsed_time;

    /* --- INITIALISATION --- */
    start_time = clock();

    current_beam[0] = current;
    current_beam[0].score = heatmap[current.y][current.x];
    
    best_ever = current;
    best_ever_heatmap = heatmap[current.y][current.x];

    /* On met une profondeur theorique tres grande (ex: 200) */
    for (depth = 0; depth < MAX_DEPTH; depth++) {
        
        /* --- SECURITE TEMPORELLE --- */
        elapsed_time = ((double)(clock() - start_time)) / CLOCKS_PER_SEC;
        /* Si on a reflechi pendant plus de 0.85 seconde, on arrete tout ! */
        if (elapsed_time > 0.85) {
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

                /* --- HEURISTIQUE UNIVERSELLE CORRIGEE --- */
                
                /* 1. Calcul du minimum vital : 1 case coute au minimum 1 unite d'essence */
                min_gas_needed = heatmap[candidate.y][candidate.x] / 100;

                /* 2. Survie absolue : on tue les futurs impossibles (malus massif) */
                if (candidate.gas < min_gas_needed) {
                    candidate.score = 500000 + heatmap[candidate.y][candidate.x];
                } else {
                    /* 3. Poids dynamique proportionnel au coussin de securite */
                    if (candidate.gas >= min_gas_needed * 2) {
                        gas_weight = 1;
                    } else {
                        buffer = min_gas_needed; 
                        if (buffer == 0) buffer = 1; 
                        
                        extra_gas = candidate.gas - min_gas_needed;
                        if (extra_gas < 0) extra_gas = 0;
                        
                        gas_weight = 50 - ((extra_gas * 49) / buffer);
                        
                        if (gas_weight < 1) gas_weight = 1;
                        if (gas_weight > 50) gas_weight = 50;
                    }

                    /* 4. Score final = Distance restante + (Consommation * Poids) */
                    candidate.score = heatmap[candidate.y][candidate.x] + ((current.gas - candidate.gas) * gas_weight);
                }
                
                /* --- NOUVEAU : Sauvegarde du record de distance (Baroud d'honneur) --- */
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

    /* Affichage optionnel de la profondeur (utile pour le debug) */
    fprintf(stderr, "    (Profondeur atteinte : %d en %.2f s)\n", depth, elapsed_time);

    /* On retourne l'action qui a permis d'atteindre le meilleur etat simule */
    return best_ever.first_action;
}

/* ========================================================================= */
/* 4. PROGRAMME PRINCIPAL                                                    */
/* ========================================================================= */

int main() {
    /* Declaration de TOUTES les variables en haut */
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
    
    /* NOUVEAU : Variables pour stocker la position précédente */
    int prev_x = -1;
    int prev_y = -1;

    prev_x = -1;
    prev_y = -1;

    fgets(line_buffer, MAX_LINE_LENGTH, stdin);
    sscanf(line_buffer, "%d %d %d", &width, &height, &gasLevel);
    
    fprintf(stderr, "=== >Map< ===\n");
    fprintf(stderr, "Size %d x %d\nGas at start %d \n\n", width, height, gasLevel);

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

    fprintf(stderr, "\n=== Race start ===\n");
    
    my_car.vx = 0;
    my_car.vy = 0;
    my_car.gas = gasLevel;
    my_car.boosts = BOOSTS_AT_START;

    while (!feof(stdin)) {
        round++;
        
        fgets(line_buffer, MAX_LINE_LENGTH, stdin);
        if (strlen(line_buffer) < 5) break; 
        
        sscanf(line_buffer, "%d %d %d %d %d %d", &myX, &myY, &secondX, &secondY, &thirdX, &thirdY);
        
        fprintf(stderr, "=== ROUND %d\n", round);
        fprintf(stderr, "    Positions: Me(%d,%d)  A(%d,%d), B(%d,%d)\n", myX, myY, secondX, secondY, thirdX, thirdY);
        
        /* --- NOUVEAU : Synchronisation de la vitesse réelle --- */
        if (round > 1) {
            my_car.vx = myX - prev_x;
            my_car.vy = myY - prev_y;
        } else {
            /* Au premier tour, on est obligatoirement à l'arrêt */
            my_car.vx = 0;
            my_car.vy = 0;
        }
        
        my_car.x = myX;
        my_car.y = myY;

        /* On sauvegarde la position actuelle pour le calcul du tour suivant */
        prev_x = myX;
        prev_y = myY;
        /* ------------------------------------------------------ */

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
        
        fprintf(stderr, "    Action: %d %d   Gas remaining: %d\n", best_action.ax, best_action.ay, my_car.gas);
        fflush(stderr);
    }

    for (row = 0; row < height; ++row) {
        free(map[row]);
        free(heatmap[row]);
    }
    free(map);
    free(heatmap);

    return EXIT_SUCCESS;
}