// --- Paramètres de dimensionnement ---
#define MAX_NEW_OBS 64
#define MAX_TRACKED_OBS 64
#define MAX_UNTRACKED_OBS 64

// ================== Types et structures de données ===================


typedef struct { 
    float x, y; 
} Point2f;


typedef struct {
    Point2f center; // centre estimé
    Point2f velocity; // vitesse estimée
    float radius; // rayon estimé (incluant la marge capteur)
    float true_radius; // rayon "réel" (corrigé de la marge) – rempli à l'export
} CircleObstacle;


typedef struct {
    // Segments ignorés ici; ne garder que les cercles (comme la node)
    CircleObstacle circles[MAX_NEW_OBS];
    uint16_t circle_count;
    // frame_id/horodatage: à gérer dans votre couche d'application
} Obstacles;


// ------------ Kalman 1D [pos; vel] ------------
typedef struct {
    // Etat x = [pos; vel]
    float pos; float vel;
    // Covariance 2x2 P
    float P00, P01, P10, P11;
} KF2;


// Kalman 1D scalaire (pour le rayon)
typedef struct {
    float x;
    float P;
} KF1;


// Tracked obstacle (équivalent TrackedObstacle C++)
typedef struct {
    CircleObstacle ob; // état courant (pour accès simple)
    KF2 kfx; // filtre axe X
    KF2 kfy; // filtre axe Y
    KF1 kfr; // filtre rayon
    int fade_counter; // compteur d'oubli
} TrackedObstacle;


// ------------------ Paramètres runtime ------------------
typedef struct {
    bool active;
    bool copy_segments; // ignoré (pas de segments)
    float loop_rate; // Hz
    float sampling_time; // s = 1/loop_rate
    float sensor_rate; // Hz (ex: 10 pour Hokuyo)
    float tracking_duration; // s (taille du fade counter)
    float min_correspondence_cost; // seuil de correspondance
    float std_correspondence_dev; // écart-type modèle de correspondance (non utilisé dans cost
    float process_variance; // q pour pos
    float process_rate_variance; // q pour vel
    float measurement_variance; // r pour mesures (x,y,r)
    float radius_margin; // marge capteur (circle.radius - true_radius)
} ObstacleTrackerParams;

// =================== État interne du module ===================


typedef struct {
    ObstacleTrackerParams p;
    // banques d'obstacles
    TrackedObstacle tracked[MAX_TRACKED_OBS];
    int T;
    CircleObstacle untracked[MAX_UNTRACKED_OBS];
    int U;
    // buffer dernière mesure reçue
    Obstacles last_measure;
    bool have_measure;
    // taille du fade counter
    int fade_counter_size;
} ObstacleTrackerCtx;


static ObstacleTrackerCtx g_ctx; // instance unique

// ======================== Utilitaires ========================
static inline float sqr(float v){ return v*v; }
static inline float length2(Point2f a, Point2f b){ return sqrtf(sqr(a.x-b.x)+sqr(a.y-b.y)); }
static inline float len(Point2f v){ return sqrtf(sqr(v.x)+sqr(v.y)); }


// Rotation d'un point (autour de l'origine)
static inline Point2f rotate_point(Point2f p, float angle){
    float c = cosf(angle), s = sinf(angle);
    Point2f r = { c*p.x - s*p.y, s*p.x + c*p.y };
    return r;
}


// ======================== Kalman helpers ========================
static void kf2_init(KF2* k, float pos, float vel, float ppos, float pvel){
    k->pos = pos; k->vel = vel;
    k->P00 = ppos; k->P01 = 0.0f; k->P10 = 0.0f; k->P11 = pvel;
}


static void kf2_predict(KF2* k, float dt, float q_pos, float q_vel){
    // x = F x, F = [[1 dt],[0 1]]
    float pos = k->pos + dt * k->vel;
    float vel = k->vel;
    // P = F P F^T + Q
    float P00 = k->P00 + dt*(k->P10 + k->P01) + dt*dt*k->P11 + q_pos;
    float P01 = k->P01 + dt*k->P11;
    float P10 = k->P10 + dt*k->P11;
    float P11 = k->P11 + q_vel;
    k->pos = pos; k->vel = vel; k->P00=P00; k->P01=P01; k->P10=P10; k->P11=P11;
}


static void kf2_correct_pos(KF2* k, float z, float r){
    // Mesure: z = [1 0] x + v
    float y = z - k->pos; // innovation
    float S = k->P00 + r; // variance innovation
    float K0 = k->P00 / S; // gain pos
    float K1 = k->P10 / S; // gain vel
    k->pos += K0*y;
    k->vel += K1*y;
    // P = (I-KH)P
    float P00 = (1.0f-K0)*k->P00;
    float P01 = (1.0f-K0)*k->P01;
    float P10 = -K1*k->P00 + k->P10;
    float P11 = -K1*k->P01 + k->P11;
    k->P00=P00; k->P01=P01; k->P10=P10; k->P11=P11;
}


static void kf1_init(KF1* k, float x, float p){ k->x=x; k->P=p; }
static void kf1_predict(KF1* k, float q){ k->P += q; }