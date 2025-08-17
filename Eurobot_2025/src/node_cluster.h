#ifndef NODE_CLUSTER_H
#define NODE_CLUSTER_H

#define MAX_NUMBER_CLUSTER 50
#define MAX_POINTS 1000

#define GAP_THRESHOLD 50 // distance en mm pour considérer qu'un point est dans le même cluster

typedef struct {
    float x[MAX_POINTS];
    float y[MAX_POINTS];
    int count;
} Cluster;


void segment_clusters(float *x, float *y, int scan_size);


#endif // NODE_CLUSTER_H