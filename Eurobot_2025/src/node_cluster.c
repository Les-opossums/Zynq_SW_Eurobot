#include "main.h"

Cluster clusters[MAX_NUMBER_CLUSTER];
int cluster_count = 0;

void segment_clusters(float *x, float *y, int scan_size) {
    cluster_count = 0;
    int current = 0;
    clusters[current].count = 0;

    for (int i = 0; i < scan_size; i++) {
        if (clusters[current].count == 0) {
            clusters[current].x[0] = x[i];
            clusters[current].y[0] = y[i];
            clusters[current].count = 1;
        } else {
            float dx = x[i] - clusters[current].x[clusters[current].count - 1];
            float dy = y[i] - clusters[current].y[clusters[current].count - 1];
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist > GAP_THRESHOLD && clusters[current].count > 0) {
                // nouveau cluster
                current++;
                cluster_count++;
                clusters[current].count = 0;
            }
            clusters[current].x[clusters[current].count] = x[i];
            clusters[current].y[clusters[current].count] = y[i];
            clusters[current].count++;
        }
    }
    cluster_count++;
}