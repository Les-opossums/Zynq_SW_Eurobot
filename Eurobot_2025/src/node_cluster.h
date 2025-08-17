#ifndef NODE_CLUSTER_H
#define NODE_CLUSTER_H

#define MAX_CLUSTERS  50
#define MAX_POINTS_CLUSTER 200

#define GAP_THRESHOLD 80.0f  // mm (ajuste selon densité LD19)

typedef struct {
    uint16_t startIndex;
    uint16_t endIndex;
    uint16_t count;

    float cx, cy;       // barycentre (mm)
    float length;       // (mm) approx: proj. min->max
    float radius;       // (mm) si cercle retenu

    // modèle ligne
    float lineTheta;    // orientation principale (rad)
    float lineRho;      // distance à l'origine (mm)
    float lineRMS;      // RMS distance orthogonale (mm)

    // modèle cercle
    float ccx, ccy;     // centre cercle (mm)
    float circleRMS;    // RMS radial (mm)

    uint8_t isCircle;
    uint8_t isWall;
} LD19Cluster;

typedef struct {
    LD19Cluster clusters[MAX_CLUSTERS];
    uint16_t count;
} LD19ClusterHandler;

typedef struct {
    // Segmentation
    float gap_eps_min_mm;     // ex: 25-40 mm
    float gap_alpha;          // ex: 1.4-1.8 (coefficient ~ r*Δθ)
    float theta_cut_deg;      // ex: 25.0 (coupe par courbure)
    uint16_t min_pts_cluster; // ex: 4-5

    // Classification ligne
    float line_rms_max;       // ex: 20 mm
    float line_lambda_ratio;  // ex: 10 (allongement PCA)

    // Classification cercle
    float circle_rms_max;     // ex: 15 mm
    float circle_r_min;       // ex: 10 mm
    float circle_r_max;       // ex: 500 mm
} LD19ClusterParams;

LD19ClusterParams LD19_DefaultClusterParams(void);

static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }

static inline float clampf(float x, float a, float b){ return x < a ? a : (x > b ? b : x); }

static inline int cut_by_gap(const LD19DataPoint* a, const LD19DataPoint* b,
                             float eps_min, float alpha) {
    float di = a->distance; // mm
    float dj = b->distance; // mm
    float dth = fabsf(b->angle - a->angle); // rad
    dth = deg2rad(dth);
    // Euclidienne au capteur (loi des cos)
    float c = cosf(dth);
    float e2 = di*di + dj*dj - 2.0f*di*dj*c;    // e^2
    float tau = fmaxf(eps_min, alpha * fminf(di, dj) * dth); // mm
    return (e2 > tau * tau);
}

static inline int cut_by_curvature(const LD19DataPoint* p0,
                                   const LD19DataPoint* p1,
                                   const LD19DataPoint* p2,
                                   float theta_cut_rad) {
    // angle entre vecteurs p1->p0 et p1->p2
    float u_x = p0->x - p1->x, u_y = p0->y - p1->y;
    float v_x = p2->x - p1->x, v_y = p2->y - p1->y;
    float nu = sqrtf(u_x*u_x + u_y*u_y);
    float nv = sqrtf(v_x*v_x + v_y*v_y);
    if (nu < 1e-3f || nv < 1e-3f) return 0;
    float dot = (u_x*v_x + u_y*v_y) / (nu*nv);
    dot = clampf(dot, -1.0f, 1.0f);
    float ang = acosf(dot); // rad
    return (ang > theta_cut_rad);
}

void LD19_SegmentAdaptive(LD19Instance *inst,
                          LD19ClusterHandler *ch,
                          const LD19ClusterParams *params);


void LD19_ClassifyClustersRobust(LD19Instance *inst,
                                 LD19ClusterHandler *ch,
                                 const LD19ClusterParams *p);

float cluster_mean_dtheta(const LD19DataPointHandler *scan, const LD19Cluster *c);


#endif // NODE_CLUSTER_H