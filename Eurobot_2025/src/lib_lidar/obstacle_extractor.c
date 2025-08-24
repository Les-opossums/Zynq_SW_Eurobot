#include "../main.h"

OE_Params oe_default_params(void) {
    OE_Params p;
    p.use_split_and_merge = 1;
    p.circles_from_visibles = 1;
    p.discard_converted_segments = 1;


    p.min_group_points = 15;
    p.max_group_distance = 0.15f; 
    p.distance_proportion = 0.006f; /* ≈ 2*pi / 1000 */
    p.max_split_distance = 0.08f;
    p.max_merge_separation = 0.08f;
    p.max_merge_spread = 0.10f;


    p.max_circle_radius = 1.0f;
    p.radius_enlargement = 0.02f;


    p.min_x_limit = -1.0f; p.max_x_limit = 1.0f;
    p.min_y_limit = -1.0f; p.max_y_limit = 1.0f;
    return p;
}

/* Distance from point to line segment AB (true distance along segment hull) */
float oe_point_to_segment_dist(OE_Point a, OE_Point b, OE_Point p){
    OE_Point ab = oe_sub(b,a);
    float ab2 = oe_dot(ab,ab);
    if (ab2 <= 1e-9f) {
        return oe_len(oe_sub(p,a));
    }
    float t = oe_dot(oe_sub(p,a), ab) / ab2;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    OE_Point proj = oe_add(a, oe_scale(ab, t));
    return oe_len(oe_sub(p, proj));
}      

/* Fit a line to a set of points by IEPF: initial line = first..last, split if needed
(This function only returns the final segment endpoints; it does not recursively
add sub-segments; recursion is handled outside.) */
void oe_iepf_fit_segment(const OE_Point* pts, int b, int e, OE_Segment* seg){
    seg->first_point = pts[b];
    seg->last_point = pts[e];
}

/* Orthogonal distance from point to infinite line (A=first,B=last) */
float oe_point_to_line_dist(OE_Point a, OE_Point b, OE_Point p){
    OE_Point ab = oe_sub(b,a);
    float L = oe_len(ab);
    if (L <= 1e-9f){
        return oe_len(oe_sub(p,a));
    }
    OE_Point n = { -ab.y/L, ab.x/L };
    return fabsf( oe_dot( oe_sub(p,a), n ) );
}

/* Fit a circle with the algebraic Kåsa method. Returns false if ill-conditioned. */
uint8_t oe_fit_circle_kasa(const OE_Point* pts, const OE_PointSet* psets, int ps_begin, int ps_count, OE_Point* center, float* radius){
    double Sx=0, Sy=0, Sxx=0, Syy=0, Sxy=0, Szzz=0, Sxxp=0, Syyp=0; /* (
    The classic linear system for x^2 + y^2 + Ax + By + C = 0 is:
    [Sxx Sxy Sx] [A] = [Sx(x^2+y^2)]
    [Sxy Syy Sy] [B] [Sy(x^2+y^2)]
    [Sx Sy N ] [C] [Σ(x^2+y^2)]
    We'll accumulate the sums then solve 3x3 via Cramer's rule.
    ) */
    int N = 0;
    for (int k=0; k < ps_count; ++k){
        const OE_PointSet* ps = &psets[ps_begin+k];
        for (int i=ps->begin_idx + 5; i<=ps->end_idx - 5; ++i){
            float x = pts[i].x, y = pts[i].y;
            double z = (double)x*(double)x + (double)y*(double)y; // r^2
            Sx += x; 
            Sy += y; 
            Sxx += x*x; 
            Syy += y*y; 
            Sxy += x*y;
            Sxxp+= x*z; 
            Syyp+= y*z; 
            Szzz+= z; 
            N++;
        }
    }
    if (N < 3) return 1;
    /* Construct matrices */
    double M[3][3] = {
        {Sxx, Sxy, Sx},
        {Sxy, Syy, Sy},
        {Sx, Sy, (double)N}
    };
    double Bv[3] = { Sxxp, Syyp, Szzz };


    /* Determinant helper */
    #define DET3(a,b,c,d,e,f,g,h,i) ((a)*((e)*(i)-(f)*(h)) - (b)*((d)*(i)-(f)*(g)) + (c)*((d)*(h)-(e)*(g)))
    double D = DET3(M[0][0],M[0][1],M[0][2], M[1][0],M[1][1],M[1][2], M[2][0],M[2][1],M[2][2]);
    if (fabs(D) < 1e-12) return 0;
    double Dx = DET3(Bv[0],M[0][1],M[0][2], Bv[1],M[1][1],M[1][2], Bv[2],M[2][1],M[2][2]);
    double Dy = DET3(M[0][0],Bv[0],M[0][2], M[1][0],Bv[1],M[1][2], M[2][0],Bv[2],M[2][2]);
    double Dc = DET3(M[0][0],M[0][1],Bv[0], M[1][0],M[1][1],Bv[1], M[2][0],M[2][1],Bv[2]);
    #undef DET3


    double A = Dx/D, B = Dy/D, C = Dc/D;
    double cx = A*0.5, cy = B*0.5;
    double r2 = cx*cx + cy*cy - C;
    if (r2 <= 0.0) return 0;

    *center = (OE_Point){ (float)cx, (float)cy };
    *radius = (float)sqrt(r2);
    return 1;
}

/* Add a point set to output (returns index), with bound check */
int oe_push_pointset(OE_Output* out, OE_PointSet ps){
    if (out->num_point_sets >= OE_MAX_POINTSETS) return -1;
    out->point_sets[out->num_point_sets] = ps;
    return out->num_point_sets++;
}


/* Add segment */
int oe_push_segment(OE_Output* out, OE_Segment s){
    if (out->num_segments >= OE_MAX_SEGMENTS) return -1;
    out->segments[out->num_segments] = s;
    return out->num_segments++;
}


/* Add circle */
int oe_push_circle(OE_Output* out, OE_Circle c){
    if (out->num_circles >= OE_MAX_CIRCLES) return -1;
    out->circles[out->num_circles] = c;
    return out->num_circles++;
}

/* ---------------- grouping (build point sets & visibility) ---------------- */
void oe_group_points(const OE_Point* pts, int n, const OE_Params* P, OE_Output* out){
    const float sin_dp = sinf(2.0f * P->distance_proportion);
    if (n <= 0) return;

    OE_PointSet cur = {0, 0, 1, 1};
    for (int i=1; i<n; ++i){
        float rx = pts[i].x, ry = pts[i].y;
        float r = sqrtf(rx*rx + ry*ry);
        OE_Point prevp = pts[cur.end_idx];
        float dx = pts[i].x - prevp.x, dy = pts[i].y - prevp.y;
        float distance = sqrtf(dx*dx + dy*dy);
        if (distance < P->max_group_distance + r * P->distance_proportion){
            cur.end_idx = i; 
            cur.num_points++;
        } else {
            /* visibility test using Heron's formula to get sine of beam angle */
            float prev_r = sqrtf(prevp.x*prevp.x + prevp.y*prevp.y);
            float p = (r + prev_r + distance) * 0.5f;
            float S2 = fmaxf(p*(p-r)*(p-prev_r)*(p-distance), 0.0f);
            float S = sqrtf(S2);
            float sin_d = (r>1e-6f && prev_r>1e-6f) ? (2.0f*S/(r*prev_r)) : 0.0f;
            if (fabsf(sin_d) < sin_dp && r < prev_r) {
                cur.is_visible = 0;
            }
            /* finalize current set */
            if (cur.num_points >= P->min_group_points){ 
                oe_push_pointset(out, cur); 
            }
            /* start a new set */
            cur.begin_idx = i; cur.end_idx = i; cur.num_points = 1;
            cur.is_visible = (fabsf(sin_d) > sin_dp || r < prev_r);
        }
    }
    if (cur.num_points >= P->min_group_points){ 
        oe_push_pointset(out, cur); 
    }
}

/* ---------------- split & merge segmentation ---------------- */
void oe_detect_segments_rec(const OE_Point* pts, const OE_Params* P, OE_Output* out, OE_PointSet ps){
    if (ps.num_points < P->min_group_points) return;

    OE_Segment s; 
    oe_iepf_fit_segment(pts, ps.begin_idx, ps.end_idx, &s);

    /* find split point using distance to infinite line */
    float maxd = 0.0f; int split_idx = -1; int idx = 0;
    for (int i=ps.begin_idx; i<=ps.end_idx; ++i, ++idx){
        float d = oe_point_to_line_dist(s.first_point, s.last_point, pts[i]);
        if (d >= maxd){
            float x = pts[i].x, y = pts[i].y;
            float r = sqrtf(x*x + y*y);
            float thr = P->max_split_distance + r * P->distance_proportion;
            if (d > thr){ 
                maxd = d; 
                split_idx = i; 
            }
        }
    }


    if (P->use_split_and_merge && split_idx >= 0){
    /* ensure both subsets are large enough */
    int n1 = split_idx - ps.begin_idx + 1;
    int n2 = ps.end_idx - split_idx + 1;
        if (n1 > P->min_group_points && n2 > P->min_group_points){
            /* clone split point into both */
            OE_PointSet ps1 = { ps.begin_idx, split_idx, n1, ps.is_visible };
            OE_PointSet ps2 = { split_idx, ps.end_idx, n2, ps.is_visible };
            oe_detect_segments_rec(pts, P, out, ps1);
            oe_detect_segments_rec(pts, P, out, ps2);
            return;
        }
    }


    /* keep the segment */
    s.ps_begin = oe_push_pointset(out, ps);
    s.ps_count = (s.ps_begin>=0) ? 1 : 0;
    oe_push_segment(out, s);
}


void oe_detect_segments(const OE_Point* pts, const OE_Params* P, OE_Output* out){
    int Nps = out->num_point_sets;
    for (int i=0; i<Nps; ++i){
        oe_detect_segments_rec(pts, P, out, out->point_sets[i]);
    }
}

/* proximity test (similar to trueDistanceTo on endpoints) */
uint8_t oe_segments_prox(const OE_Segment* a, const OE_Segment* b, const OE_Params* P){
    float t = P->max_merge_separation;
    return (oe_point_to_segment_dist(a->first_point, a->last_point, b->first_point) < t ||
            oe_point_to_segment_dist(a->first_point, a->last_point, b->last_point) < t ||
            oe_point_to_segment_dist(b->first_point, b->last_point, a->first_point) < t ||
            oe_point_to_segment_dist(b->first_point, b->last_point, a->last_point) < t);
}


/* collinearity: distance of each endpoint to merged best-fit line < spread */
uint8_t oe_segments_collinear(OE_Segment merged, const OE_Segment* s1, const OE_Segment* s2, const OE_Params* P){
    float m = P->max_merge_spread;
    return (oe_point_to_line_dist(merged.first_point, merged.last_point, s1->first_point) < m &&
            oe_point_to_line_dist(merged.first_point, merged.last_point, s1->last_point) < m &&
            oe_point_to_line_dist(merged.first_point, merged.last_point, s2->first_point) < m &&
            oe_point_to_line_dist(merged.first_point, merged.last_point, s2->last_point) < m);
}


/* Fit segment over concatenated point sets by endpoints (IEPF style) */
OE_Segment oe_fit_segment_over_ps(const OE_Point* pts, const OE_PointSet* psets, int ps_begin, int ps_count){
    int b = psets[ps_begin].begin_idx;
    int e = psets[ps_begin + ps_count - 1].end_idx;
    OE_Segment s; s.ps_begin = ps_begin; s.ps_count = ps_count; s.first_point = pts[b]; s.last_point = pts[e];
    return s;
}


void oe_merge_segments(const OE_Point* pts, const OE_Params* P, OE_Output* out){
    for (int i=0; i<out->num_segments; ++i){
        for (int j=i+1; j<out->num_segments; ++j){
            OE_Segment *s1 = &out->segments[i], *s2 = &out->segments[j];
            /* enforce CCW order as in ROS code (based on cross sign) */
            OE_Point f1 = s1->first_point, f2 = s2->first_point;
            if (oe_crossz(f1, f2) < 0) { 
                OE_Segment* tmp=s1; s1=s2; s2=tmp; 
            }
            if (!oe_segments_prox(s1, s2, P)) continue;
            /* merge point sets */
            int base = out->num_point_sets;
            for (int k=0; k<s1->ps_count; ++k) {
                oe_push_pointset(out, out->point_sets[s1->ps_begin+k]);
            }
            for (int k=0; k<s2->ps_count; ++k) {
                oe_push_pointset(out, out->point_sets[s2->ps_begin+k]);
            }
            int ps_count = s1->ps_count + s2->ps_count;
            OE_Segment merged = oe_fit_segment_over_ps(pts, out->point_sets, base, ps_count);
            if (oe_segments_collinear(merged, s1, s2, P)){
                /* replace i with merged, remove j */
                out->segments[i] = merged;
                /* shift segments left from j+1 */
                for (int m=j; m<out->num_segments-1; ++m) {
                    out->segments[m] = out->segments[m+1];
                }
                out->num_segments--; j = i; /* re-check new segment against others */
            } else {
            /* rollback appended pointsets if not merged: simply ignore; harmless but can grow */
            }
        }
    }
}


void oe_detect_circles(const OE_Point* pts, const OE_Params* P, OE_Output* out){
    for (int si=0; si<out->num_segments; ++si){
        OE_Segment* s = &out->segments[si];
        uint8_t ok_visible = 1;
        if (P->circles_from_visibles){
            for (int k=0; k<s->ps_count; ++k){
                const OE_PointSet* ps = &out->point_sets[s->ps_begin+k];
                if (!ps->is_visible){ 
                    ok_visible = 0; 
                    break; 
                }
            }
            if (!ok_visible) continue;
        }
        /* Fit circle over all contributing point sets */
        OE_Point c; float r;
        if (!oe_fit_circle_kasa(pts, out->point_sets, s->ps_begin, s->ps_count, &c, &r)) continue;
        float enlarged = r + P->radius_enlargement;
        if (enlarged < P->max_circle_radius){
            OE_Circle C = { c, enlarged, r, s->ps_begin, s->ps_count };
            oe_push_circle(out, C);
            if (P->discard_converted_segments){
                /* delete this segment */
                for (int m=si; m<out->num_segments-1; ++m) {
                    out->segments[m] = out->segments[m+1];
                }
                out->num_segments--; si--; /* stay at this index */
            }
        }
    }
}


uint8_t oe_compare_circles(const OE_Circle* a, const OE_Circle* b, const OE_Params* P, OE_Circle* out_merged){
    if (a==b) return 0; // false 
    OE_Point d = oe_sub(b->center, a->center);
    float dist = oe_len(d);
    /* containment */
    if (b->radius - a->radius >= dist){ 
        *out_merged = *b; return 1; 
    }
    if (a->radius - b->radius >= dist){ 
        *out_merged = *a; return 1; 
    }
    /* overlap -> merge, create circle whose center lies proportionally between centers */
    if (a->radius + b->radius >= dist){
        float t = (a->radius)/(a->radius + b->radius);
        OE_Point center = oe_add(a->center, oe_scale(d, t));
        float radius = oe_len(oe_sub(a->center, center)) + a->radius; /* as in ROS */
        OE_Circle c = { center, radius + fmaxf(a->radius, b->radius), radius, 0, 0 };
        if (c.radius < P->max_circle_radius){ 
            *out_merged = c; return 1; 
        }
    }
    return 0;
}

void oe_merge_circles(const OE_Params* P, OE_Output* out){
    for (int i=0; i<out->num_circles; ++i){
        for (int j=i+1; j<out->num_circles; ++j){
            OE_Circle m; if (oe_compare_circles(&out->circles[i], &out->circles[j], P, &m)){
                out->circles[i] = m;
                for (int k=j; k<out->num_circles-1; ++k) {
                    out->circles[k] = out->circles[k+1];
                }
                out->num_circles--; j = i; /* re-check */
            }
        }
    }
}

/* -------------------------- main pipeline -------------------------- */
void oe_process_points(const OE_Point* pts, int n, const OE_Params* P, const OE_Transform* opt_tf, OE_Output* out){

    /* stage 1: grouping */
    oe_group_points(pts, n, P, out);
    /* stage 2: segments (split & merge + merging) */
    oe_detect_segments(pts, P, out);
    oe_merge_segments(pts, P, out);
    /* stage 3: circles (fit + merge) */
    oe_detect_circles(pts, P, out);
    oe_merge_circles(P, out);


    /* final: clamp circles by XY limits and apply transform to outputs */
    int wr = 0;
    for (int i=0; i<out->num_circles; ++i){
        OE_Circle c = out->circles[i];
        OE_Point tc = oe_rot_apply(opt_tf, c.center);
        if (tc.x > P->min_x_limit && tc.x < P->max_x_limit && tc.y > P->min_y_limit && tc.y < P->max_y_limit){
            c.center = tc; 
            out->circles[wr++] = c;
        }
    }
    out->num_circles = wr;

    for (int i=0; i<out->num_segments; ++i){
        out->segments[i].first_point = oe_rot_apply(opt_tf, out->segments[i].first_point);
        out->segments[i].last_point = oe_rot_apply(opt_tf, out->segments[i].last_point);
    }
}