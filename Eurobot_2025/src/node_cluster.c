#include "main.h"

void LD19_SegmentAdaptive(LD19Instance *inst,
                          LD19ClusterHandler *ch,
                          const LD19ClusterParams *params)
{
    ch->count = 0;
    if (!inst || !inst->currentScan) return;

    LD19DataPointHandler *scan = inst->currentScan;
    const int N = (int)scan->index;
    if (N < 2) return;

    const float theta_cut_rad = params->theta_cut_deg * (float)M_PI / 180.0f;

    int clusterStart = 0;
    for (int i = 1; i < N; i++) {
        int cut = 0;

        // 1) gap adaptatif
        if (cut_by_gap(&scan->points[i-1], &scan->points[i],
                       params->gap_eps_min_mm, params->gap_alpha)) {
            cut = 1;
        }

        int curv = 0;
        if (i+1 < N && i-1 >= 1) {
            curv = cut_by_curvature(&scan->points[i-1], &scan->points[i], &scan->points[i+1], theta_cut_rad);
        }
        // on exige gap OU (courbure ET grand gap local moins strict)
        if (!cut && curv) {
            // petit relâchement : coupe si distance euclidienne > eps_min seulement
            float di = scan->points[i-1].distance;
            float dj = scan->points[i].distance;
            float dth = fabsf(scan->points[i].angle - scan->points[i-1].angle);
            // convertir si besoin :
            dth = deg2rad(dth);
            float e2 = di*di + dj*dj - 2.0f*di*dj*cosf(dth);
            if (e2 > (params->gap_eps_min_mm * params->gap_eps_min_mm)) {
                cut = 1;
            }
        }

        if (cut) {
            if (ch->count < MAX_CLUSTERS) {
                LD19Cluster *c = &ch->clusters[ch->count++];
                c->startIndex = (uint16_t)clusterStart;
                c->endIndex   = (uint16_t)(i-1);
                c->count      = c->endIndex - c->startIndex + 1;
            }
            clusterStart = i;
        }
    }

    // dernier cluster
    if (clusterStart < N && ch->count < MAX_CLUSTERS) {
        LD19Cluster *c = &ch->clusters[ch->count++];
        c->startIndex = (uint16_t)clusterStart;
        c->endIndex   = (uint16_t)(N-1);
        c->count      = c->endIndex - c->startIndex + 1;
    }

    // filtre clusters trop petits
    uint16_t w = 0;
    for (uint16_t k = 0; k < ch->count; k++) {
        if (ch->clusters[k].count >= params->min_pts_cluster) {
            if (w != k) ch->clusters[w] = ch->clusters[k];
            w++;
        }
    }
    ch->count = w;
}

static void cluster_centroid(const LD19DataPointHandler *scan,
                             LD19Cluster *c) {
    float sx=0, sy=0;
    for (uint16_t i = c->startIndex; i <= c->endIndex; i++) {
        sx += scan->points[i].x;
        sy += scan->points[i].y;
    }
    float inv = 1.0f / (float)c->count;
    c->cx = sx * inv;
    c->cy = sy * inv;
}

static void cluster_covariance(const LD19DataPointHandler *scan,
                               const LD19Cluster *c,
                               float *a, float *b, float *d) {
    // Cov = [a b; b d]
    float cx=c->cx, cy=c->cy;
    float sxx=0, sxy=0, syy=0;
    for (uint16_t i=c->startIndex;i<=c->endIndex;i++){
        float dx = scan->points[i].x - cx;
        float dy = scan->points[i].y - cy;
        sxx += dx*dx; sxy += dx*dy; syy += dy*dy;
    }
    // on normalise par N (ou N-1, l’échelle n’affecte pas le ratio)
    float n = (float)c->count;
    *a = sxx / n; *b = sxy / n; *d = syy / n;
}

static void fit_line_tls(const LD19DataPointHandler *scan, LD19Cluster *c) {
    float a,b,d;
    cluster_centroid(scan, c);
    cluster_covariance(scan, c, &a,&b,&d);

    // angle principal de la droite: theta = 0.5 atan2(2b, a - d)
    float theta = 0.5f * atan2f(2.0f*b, a - d);

    c->lineTheta = theta;

    // vecteur normal n = (-sinθ, cosθ), distance rho = n·centroid
    float n_x = -sinf(theta), n_y = cosf(theta);
    c->lineRho = n_x * c->cx + n_y * c->cy;

    // RMS orthogonale (≈ √λ2)
    float tmp = sqrtf((a - d)*(a - d) + 4.0f*b*b);
    float lambda1 = 0.5f * (a + d + tmp);  // var le long
    float lambda2 = 0.5f * (a + d - tmp);  // var orthogonale
    if (lambda2 < 0) lambda2 = 0;
    c->lineRMS = sqrtf(lambda2);

    // longueur approximée = étendue projetée sur axe principal
    float u_x = cosf(theta), u_y = sinf(theta);
    float minp=1e9f, maxp=-1e9f;
    for (uint16_t i=c->startIndex;i<=c->endIndex;i++){
        float px = scan->points[i].x;
        float py = scan->points[i].y;
        float t = (px - c->cx)*u_x + (py - c->cy)*u_y;
        if (t < minp) minp = t;
        if (t > maxp) maxp = t;
    }
    c->length = (maxp - minp);  // mm
}

static int solve3x3(float M[3][3], float y[3], float v[3]) {
    // Gauss-Jordan sans pivot élaboré (ok pour n petit, données propres)
    for (int i=0;i<3;i++){
        // pivot
        float piv = M[i][i];
        if (fabsf(piv) < 1e-8f) return 0;
        float inv = 1.0f/piv;
        for (int j=i;j<3;j++) M[i][j]*=inv;
        y[i]*=inv;

        for (int k=0;k<3;k++){
            if (k==i) continue;
            float f = M[k][i];
            for (int j=i;j<3;j++) M[k][j]-=f*M[i][j];
            y[k]-=f*y[i];
        }
    }
    v[0]=y[0]; v[1]=y[1]; v[2]=y[2];
    return 1;
}

static void fit_circle_kasa(const LD19DataPointHandler *scan, LD19Cluster *c) {
    cluster_centroid(scan, c); // pas obligatoire pour Kåsa, mais utile ailleurs

    float Sx=0,Sy=0,Sxx=0,Syy=0,Sxy=0,Sxxx=0,Sxxy=0,Sxyy=0,Syyy=0;
    for (uint16_t i=c->startIndex;i<=c->endIndex;i++){
        float x = scan->points[i].x;
        float y = scan->points[i].y;
        float xx = x*x, yy = y*y;
        Sx+=x; Sy+=y; Sxx+=xx; Syy+=yy; Sxy+=x*y;
        Sxxx+=xx*x; Sxxy+=x*x*y; Sxyy+=x*y*y; Syyy+=yy*y;
    }
    float N = (float)c->count;

    // Formulation: x^2 + y^2 + a x + b y + c = 0
    // Linéarisation classique
    float M[3][3] = {
        {2*Sxx, 2*Sxy, 2*Sx},
        {2*Sxy, 2*Syy, 2*Sy},
        {2*Sx , 2*Sy , 2*N}
    };
    float Y[3] = { Sxxx + Sxyy, Sxxy + Syyy, Sxx + Syy };
    float V[3];

    int ok = solve3x3(M, Y, V);
    if (!ok) { c->ccx=c->ccy=0; c->radius=0; c->circleRMS=1e9f; return; }

    float a = V[0], b = V[1], cc = V[2];
    c->ccx = -a*0.5f;
    c->ccy = -b*0.5f;
    float r2 = c->ccx*c->ccx + c->ccy*c->ccy - cc;
    c->radius = (r2 > 0) ? sqrtf(r2) : 0.0f;

    // RMS radial
    float err=0;
    for (uint16_t i=c->startIndex;i<=c->endIndex;i++){
        float dx = scan->points[i].x - c->ccx;
        float dy = scan->points[i].y - c->ccy;
        float d  = sqrtf(dx*dx+dy*dy);
        float e  = d - c->radius;
        err += e*e;
    }
    c->circleRMS = sqrtf(err / N);
}

void LD19_ClassifyClustersRobust(LD19Instance *inst,
                                 LD19ClusterHandler *ch,
                                 const LD19ClusterParams *p)

                                 
{
    LD19DataPointHandler *scan = inst->currentScan;

    for (uint16_t k=0; k<ch->count; k++){
        LD19Cluster *c = &ch->clusters[k];
        c->isCircle = 0; c->isWall = 0;

        // Fit ligne (TLS/PCA)
        fit_line_tls(scan, c);

        // Allongement PCA: ratio = var_long / var_orth ≈ (λ1 / λ2)
        // On le retrouve via lineRMS (≈√λ2) mais il faudrait aussi √λ1.
        // Approximons λ1 = totalVar - λ2 :
        float a,b,d;
        cluster_covariance(scan, c, &a,&b,&d);
        float tmp = sqrtf((a - d)*(a - d) + 4.0f*b*b);
        float lambda1 = 0.5f * (a + d + tmp);
        float lambda2 = 0.5f * (a + d - tmp);
        if (lambda2 < 1e-9f) lambda2 = 1e-9f;
        float elong = lambda1 / lambda2;

        // Fit cercle
        fit_circle_kasa(scan, c);

        float mean_dth = cluster_mean_dtheta(scan, c);
        // si angle en DEGRÉS => convertir ici :
        mean_dth = deg2rad(mean_dth);

        // Heuristiques robustes
        int wall_ok = (c->lineRMS <= p->line_rms_max) &&
                      (elong >= p->line_lambda_ratio) &&
                      (c->length >= 120.0f); // mur : au moins ~12 cm

        int circle_ok = (c->circleRMS <= p->circle_rms_max) &&
                        (c->radius >= p->circle_r_min) &&
                        (c->radius <= p->circle_r_max);

        // Si les deux passent, on choisit le plus “explicatif” (RMS plus petit)
        if (wall_ok && circle_ok) {
            if (c->circleRMS < c->lineRMS) { c->isCircle=1; }
            else                           { c->isWall=1;   }
        } else if (wall_ok) {
            c->isWall = 1;
        } else if (circle_ok) {
            c->isCircle = 1;
        }
    }
}

LD19ClusterParams LD19_DefaultClusterParams(void) {
    LD19ClusterParams p = {
        .gap_eps_min_mm    = 35.0f,  
        .gap_alpha         = 1.7f,   
        .theta_cut_deg     = 50.0f,  
        .min_pts_cluster   = 5,      

        .line_rms_max      = 25.0f,
        .line_lambda_ratio = 8.0f,

        .circle_rms_max    = 25.0f,  
        .circle_r_min      = 10.0f,
        .circle_r_max      = 800.0f  
    };
    return p;
}

float cluster_mean_dtheta(const LD19DataPointHandler *scan, const LD19Cluster *c){
    if (c->count < 2) return 0.0f;
    float s=0.0f; int m=0;
    for (uint16_t i=c->startIndex+1; i<=c->endIndex; i++){
        float dth = fabsf(scan->points[i].angle - scan->points[i-1].angle);
        s += dth; m++;
    }
    return (m>0) ? s / (float)m : 0.0f;
}
