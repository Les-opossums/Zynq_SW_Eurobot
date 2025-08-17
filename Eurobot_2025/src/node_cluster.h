#ifndef NODE_CLUSTER_H
#define NODE_CLUSTER_H

#define MAX_CLUSTERS  50
#define MAX_POINTS_CLUSTER 200

#define GAP_THRESHOLD 80.0f  // mm (ajuste selon densité LD19)

typedef struct {
    uint16_t startIndex;       // index du premier point dans le scan
    uint16_t endIndex;         // index du dernier point dans le scan
    uint16_t count;            // nombre de points dans le cluster
    float cx;                  // barycentre X (mm)
    float cy;                  // barycentre Y (mm)
    float length;              // longueur (mm)
    float radius;              // rayon moyen (mm)
    uint8_t isCircle;          // 1 si cluster ≈ cercle
    uint8_t isWall;            // 1 si cluster ≈ mur
} LD19Cluster;

typedef struct {
    LD19Cluster clusters[MAX_CLUSTERS];
    uint16_t count;
} LD19ClusterHandler;


void LD19_FindClusters(LD19Instance *inst, LD19ClusterHandler *ch);
void LD19_ClassifyClusters(LD19Instance *inst, LD19ClusterHandler *ch);


#endif // NODE_CLUSTER_H