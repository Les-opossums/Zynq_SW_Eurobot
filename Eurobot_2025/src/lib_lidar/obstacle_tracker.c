#include "../main.h"


void obstacle_tracker_init(const ObstacleTrackerParams* p){
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.p = *p;
    g_ctx.p.sampling_time = (p->loop_rate > 0.0f) ? (1.0f/p->loop_rate) : 0.01f;
    g_ctx.fade_counter_size = (int)(g_ctx.p.loop_rate * g_ctx.p.tracking_duration);
    ctx_reset_lists(&g_ctx);
}


// =================== Coût de correspondance ===================
static float obstacle_cost(const CircleObstacle* n, const CircleObstacle* o, float sensor_rate){
    // Coût simple: distance dans (x,y,r)
    float dx = n->center.x - o->center.x;
    float dy = n->center.y - o->center.y;
    float dr = n->radius - o->radius;
    float cost = sqrtf(dx*dx + dy*dy + dr*dr);
    (void)sensor_rate; // pénalité directionnelle ignorée (comme code d'origine retournant cost/1)
    return cost;
}


// ===================== Matching + Fusion/Fission =====================


// Calcule les indices minima par ligne/colonne
static void compute_row_min_indices(const float* cost, int N, int M, float min_cost, int* row_min){
    // cost est une matrice N x M (rangée majeure)
    for(int n=0;n<N;++n){
        float best = min_cost; int best_idx = -1;
        const float* row = &cost[n*M];
        for(int m=0;m<M;++m){ 
            if(row[m] < best){ 
                best = row[m]; best_idx = m; 
            } 
        }
        row_min[n] = best_idx; // -1 si aucune
    }
}


static void compute_col_min_indices(const float* cost, int N, int M, float min_cost, int* col_min){
    for(int m=0;m<M;++m){
        float best = min_cost; int best_idx = -1;
        for(int n=0;n<N;++n){ 
            float v = cost[n*M + m]; 
            if(v < best){  
                best = v; 
                best_idx = n; 
            }
        }
        col_min[m] = best_idx; // -1 si aucune
    }
}


// ======================= API: Nouveau lot de mesures =======================


void obstacle_tracker_on_new_obstacles(const Obstacles* new_obs){
// Déduire radius_margin si possible
}