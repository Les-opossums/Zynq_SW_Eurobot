#include "main.h"

void LD19_FindClusters(LD19Instance *inst, LD19ClusterHandler *ch) {
    ch->count = 0;
    if (!inst || !inst->previousScan) return;

    LD19DataPointHandler *scan = inst->previousScan;
    if (scan->index < 2) return;

    int clusterStart = 0;
    for (int i = 1; i < scan->index; i++) {
        float dx = scan->points[i].x - scan->points[i-1].x;
        float dy = scan->points[i].y - scan->points[i-1].y;
        float dist = sqrtf(dx*dx + dy*dy);

        if (dist > GAP_THRESHOLD) {
            if (ch->count < MAX_CLUSTERS) {
                LD19Cluster *c = &ch->clusters[ch->count++];
                c->startIndex = clusterStart;
                c->endIndex   = i-1;
                c->count      = c->endIndex - c->startIndex + 1;
            }
            clusterStart = i;
        }
    }

    // dernier cluster
    if (ch->count < MAX_CLUSTERS && clusterStart < scan->index) {
        LD19Cluster *c = &ch->clusters[ch->count++];
        c->startIndex = clusterStart;
        c->endIndex   = scan->index - 1;
        c->count      = c->endIndex - c->startIndex + 1;
    }
}

void LD19_ClassifyClusters(LD19Instance *inst, LD19ClusterHandler *ch) {
    LD19DataPointHandler *scan = inst->previousScan;

    for (int k = 0; k < ch->count; k++) {
        LD19Cluster *c = &ch->clusters[k];
        c->isCircle = 0;
        c->isWall   = 0;

        // barycentre
        float cx = 0, cy = 0;
        for (int i = c->startIndex; i <= c->endIndex; i++) {
            cx += scan->points[i].x;
            cy += scan->points[i].y;
        }
        cx /= c->count;
        cy /= c->count;
        c->cx = cx;
        c->cy = cy;

        // rayon moyen et variance
        float r = 0, rvar = 0;
        float maxDist = 0;
        for (int i = c->startIndex; i <= c->endIndex; i++) {
            float dx = scan->points[i].x - cx;
            float dy = scan->points[i].y - cy;
            float d = sqrtf(dx*dx + dy*dy);
            r += d;
            if (d > maxDist) maxDist = d;
        }
        r /= c->count;
        for (int i = c->startIndex; i <= c->endIndex; i++) {
            float dx = scan->points[i].x - cx;
            float dy = scan->points[i].y - cy;
            float d = sqrtf(dx*dx + dy*dy);
            rvar += (d-r)*(d-r);
        }
        rvar /= c->count;

        c->radius = r;
        c->length = maxDist * 2.0f;

        // heuristiques
        if (c->count < 15 && sqrtf(rvar) < 60.0f) { 
            c->isCircle = 1;  // petit objet rond
        }
        if (c->count >= 20 && c->length > 100.0f) {
            c->isWall = 1;    // mur ou grand objet allongé
        }
    }
}