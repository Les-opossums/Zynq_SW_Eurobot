#ifndef OBSTACLE_EXTRACTOR_H
#define OBSTACLE_EXTRACTOR_H

//* ----------- Tunable compile-time limits ----------- */
#define OE_MAX_POINTS 4096
#define OE_MAX_POINTSETS 1024
#define OE_MAX_SEGMENTS 256
#define OE_MAX_CIRCLES 128

//* ----------- Data structures ----------- */
typedef struct { 
    float x, y; 
} OE_Point;

/* A contiguous subset of the input points */
typedef struct {
    int begin_idx; /* inclusive index in input array */
    int end_idx; /* inclusive index in input array */
    int num_points; /* = end_idx - begin_idx + 1 */
    uint8_t is_visible; /* per the sine-angle heuristic  1 = visible, 0 = not visible */
} OE_PointSet;

/* Segment represented by its endpoints (counter-clockwise order assumed) */
typedef struct {
    OE_Point first_point;
    OE_Point last_point;
    /* bookkeeping: which point sets formed this segment */
    int ps_begin; /* index into out.point_sets[] */
    int ps_count; /* number of point sets */
} OE_Segment;


/* Circle obstacle */
typedef struct {
    OE_Point center;
    float radius; /* enlarged radius (used for collision) */
    float true_radius; /* fitted radius before enlargement */
    int ps_begin; /* contributing point sets */
    int ps_count;
} OE_Circle;


/* Optional simple SE2 transform for outputs (no TF tree) */
typedef struct {
    uint8_t enabled; /* if true, apply to segments and circles */
    float cos_th; /* cos(theta) */
    float sin_th; /* sin(theta) */
    float tx, ty; /* translation */
} OE_Transform;


/* Parameters (runtime) */
typedef struct {
    uint8_t use_split_and_merge; /* IEPF */
    uint8_t circles_from_visibles; /* only visible point sets */
    uint8_t discard_converted_segments;/* erase segment if converted to circle */

    int min_group_points; /* minimum points to form a set */

    float max_group_distance; /* base threshold [m] */
    float distance_proportion; /* grows with range */
    float max_split_distance; /* IEPF splitting threshold */
    float max_merge_separation; /* segment endpoint proximity */
    float max_merge_spread; /* collinearity tolerance */

    float max_circle_radius; /* discard circles larger than this */
    float radius_enlargement; /* inflate fitted radius */

    /* Output XY limits for circles */
    float min_x_limit, max_x_limit;
    float min_y_limit, max_y_limit;
} OE_Params;

/* Output buffers and counters */
typedef struct {
    OE_PointSet point_sets[OE_MAX_POINTSETS];
    int num_point_sets;

    OE_Segment segments[OE_MAX_SEGMENTS];
    int num_segments;

    OE_Circle circles[OE_MAX_CIRCLES];
    int num_circles;
} OE_Output;

/* ---- small helpers ---- */
static inline float oe_dot(OE_Point a, OE_Point b){ 
    return a.x*b.x + a.y*b.y; 
}
static inline OE_Point oe_sub(OE_Point a, OE_Point b){ 
    OE_Point r={a.x-b.x,a.y-b.y}; 
    return r; 
}
static inline OE_Point oe_add(OE_Point a, OE_Point b){ 
    OE_Point r={a.x+b.x,a.y+b.y}; 
    return r; 
}
static inline OE_Point oe_scale(OE_Point a, float s){ 
    OE_Point r={a.x*s,a.y*s}; 
    return r; 
}
static inline float oe_len(OE_Point a){ 
    return sqrtf(oe_dot(a,a)); 
}
static inline float oe_crossz(OE_Point a, OE_Point b){ 
    return a.x*b.y - a.y*b.x; 
}
static inline OE_Point oe_rot_apply(const OE_Transform* tf, OE_Point p){
    if (!tf || !tf->enabled) return p;
    OE_Point r = { tf->cos_th*p.x - tf->sin_th*p.y + tf->tx,
    tf->sin_th*p.x + tf->cos_th*p.y + tf->ty };
    return r;
}

static inline void oe_reset_output(OE_Output* out) {
    out->num_point_sets = 0;
    out->num_segments = 0;
    out->num_circles = 0;
}

OE_Params oe_default_params(void);
float oe_point_to_segment_dist(OE_Point a, OE_Point b, OE_Point p);
void oe_iepf_fit_segment(const OE_Point* pts, int b, int e, OE_Segment* seg);
float oe_point_to_line_dist(OE_Point a, OE_Point b, OE_Point p);
uint8_t oe_fit_circle_kasa(const OE_Point* pts, const OE_PointSet* psets, int ps_begin, int ps_count, OE_Point* center, float* radius);
int oe_push_pointset(OE_Output* out, OE_PointSet ps);
int oe_push_segment(OE_Output* out, OE_Segment s);
int oe_push_circle(OE_Output* out, OE_Circle c);
void oe_group_points(const OE_Point* pts, int n, const OE_Params* P, OE_Output* out);
void oe_detect_segments_rec(const OE_Point* pts, const OE_Params* P, OE_Output* out, OE_PointSet ps);
void oe_detect_segments(const OE_Point* pts, const OE_Params* P, OE_Output* out);
uint8_t oe_segments_prox(const OE_Segment* a, const OE_Segment* b, const OE_Params* P);
uint8_t oe_segments_collinear(OE_Segment merged, const OE_Segment* s1, const OE_Segment* s2, const OE_Params* P);
OE_Segment oe_fit_segment_over_ps(const OE_Point* pts, const OE_PointSet* psets, int ps_begin, int ps_count);
void oe_merge_segments(const OE_Point* pts, const OE_Params* P, OE_Output* out);
void oe_detect_circles(const OE_Point* pts, const OE_Params* P, OE_Output* out);
uint8_t oe_compare_circles(const OE_Circle* a, const OE_Circle* b, const OE_Params* P, OE_Circle* out_merged);
void oe_merge_circles(const OE_Params* P, OE_Output* out);
void oe_process_points(const OE_Point* pts, int n, const OE_Params* P, const OE_Transform* opt_tf, OE_Output* out);




#endif // OBSTACLE_EXTRACTOR_H