#include "../main.h"

OE_Params oe_default_params(void) {
    OE_Params p;
    p.use_split_and_merge = 1;
    p.circles_from_visibles = 1;
    p.discard_converted_segments = 1;

    p.min_group_points = 5;

    p.max_group_distance   = 100;   // 0.10 m
    p.distance_proportion = 6280;  // 0.00628 ≈ 2π/1000 en "×10^-6"
    p.max_split_distance   = 200;   // 0.20 m
    p.max_merge_separation = 200;
    p.max_merge_spread     = 200;

    p.max_circle_radius = 600;  // 0.60 m
    p.radius_enlargement = 250; // 0.25 m

    p.min_x_limit = -10000; p.max_x_limit = 10000;
    p.min_y_limit = -10000; p.max_y_limit = 10000;

    return p;
}

/* Distance from point to line segment AB (true distance along segment hull) */
int32_t oe_point_to_segment_dist(OE_Point a, OE_Point b, OE_Point p){
    int32_t vx = b.x - a.x, vy = b.y - a.y;
    int64_t v2 = (int64_t)vx*vx + (int64_t)vy*vy;
    if (v2 <= 0) {
        int64_t dx = (int64_t)p.x - a.x, dy = (int64_t)p.y - a.y;
        return (int32_t)sqrt((double)(dx*dx + dy*dy));
    }
    int64_t px = (int64_t)p.x - a.x, py = (int64_t)p.y - a.y;
    int64_t t_num = px*vx + py*vy; /* may be negative */
    if (t_num <= 0) {
        int64_t dx = px, dy = py; return (int32_t)sqrt((double)(dx*dx + dy*dy));
    }
    if (t_num >= v2) {
        int64_t dx = (int64_t)p.x - b.x, dy = (int64_t)p.y - b.y;
        return (int32_t)sqrt((double)(dx*dx + dy*dy));
    }
    /* projection point = A + (t_num/v2) * (B-A) ; distance to P */
    /* compute perpendicular distance via cross product magnitude |(P-A) x (B-A)| / |B-A| */
    int64_t cross = llabs(px*vy - py*vx);
    double d = (double)cross / sqrt((double)v2);
    return (int32_t)(d + 0.5);
}      

/* Fit a line to a set of points by IEPF: initial line = first..last, split if needed
(This function only returns the final segment endpoints; it does not recursively
add sub-segments; recursion is handled outside.) */
void oe_iepf_fit_segment(const OE_Point* pts, int b, int e, OE_Segment* seg){
    seg->first_point = pts[b];
    seg->last_point = pts[e];
}

/* Orthogonal distance from point to infinite line (A=first,B=last) */
int32_t oe_point_to_line_dist(OE_Point a, OE_Point b, OE_Point p){
    int32_t vx = b.x - a.x, vy = b.y - a.y;
    int64_t v2 = (int64_t)vx*vx + (int64_t)vy*vy;
    if (v2 <= 0) {
        int64_t dx = (int64_t)p.x - a.x, dy = (int64_t)p.y - a.y;
        return (int32_t)sqrt((double)(dx*dx + dy*dy));
    }
    int64_t px = (int64_t)p.x - a.x, py = (int64_t)p.y - a.y;
    int64_t cross = llabs(px*vy - py*vx);
    double d = (double)cross / sqrt((double)v2);
    return (int32_t)(d + 0.5);
}

/* Fit a circle with the algebraic Kåsa method. Returns false if ill-conditioned. */
uint8_t oe_fit_circle_kasa(const OE_Point* pts, const OE_PointSet* psets, int ps_begin, int ps_count, OE_Point* center, int32_t* radius){
    double Sx=0,Sy=0,Sxx=0,Syy=0,Sxy=0,Szz=0,Sxxp=0,Syyp=0; 
    int N=0;
    for(int k=0;k<ps_count;++k){
        const OE_PointSet* ps=&psets[ps_begin+k];
        for(int i=ps->begin_idx;i<=ps->end_idx;++i){
            double x=pts[i].x, y=pts[i].y;
            double z=x*x + y*y;
            Sx+=x; Sy+=y; Sxx+=x*x; Syy+=y*y; Sxy+=x*y; Szz+=z; Sxxp+=x*z; Syyp+=y*z; N++;
        }
    }
    if (N<3) return 0;
    double M00=Sxx, M01=Sxy, M02=Sx;
    double M10=Sxy, M11=Syy, M12=Sy;
    double M20=Sx, M21=Sy, M22=(double)N;
    double B0=Sxxp, B1=Syyp, B2=Szz;

    #define DET3(a,b,c,d,e,f,g,h,i) ((a)*((e)*(i)-(f)*(h)) - (b)*((d)*(i)-(f)*(g)) + (c)*((d)*(h)-(e)*(g)))
    double D = DET3(M00,M01,M02,M10,M11,M12,M20,M21,M22);
    if (fabs(D) < 1e-12) return 0;
    double Dx=DET3(B0,M01,M02,B1,M11,M12,B2,M21,M22);
    double Dy=DET3(M00,B0,M02, M10,B1,M12, M20,B2,M22);
    double Dc=DET3(M00,M01,B0, M10,M11,B1, M20,M21,B2);
    #undef DET3
    
    double A=Dx/D, B=Dy/D, C=Dc/D;
    double cx=-A*0.5, cy=-B*0.5; 
    double r2=cx*cx+cy*cy - C; 
    if (r2<=0) return 0;

    center->x = (int32_t)llround(cx); 
    center->y = (int32_t)llround(cy);
    *radius = (int32_t)llround(sqrt(r2));
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
    ensure_sin2dp(P);
    if (n<=0) return;

    OE_PointSet cur = {0,0,1,1};
    for (int i=1;i<n;++i){
        /* current point range */
        int64_t r2 = (int64_t)pts[i].x*pts[i].x + (int64_t)pts[i].y*pts[i].y;
        int32_t r = (int32_t)sqrt((double)r2);

        /* distance to previous point in current set */
        int32_t dx = pts[i].x - pts[cur.end_idx].x;
        int32_t dy = pts[i].y - pts[cur.end_idx].y;
        int32_t dist = (int32_t)sqrt((double)((int64_t)dx*dx + (int64_t)dy*dy));

        int32_t thr = P->max_group_distance + (int32_t)(( (int64_t)r * P->distance_proportion ) / 1000000LL);
        if (dist < thr){
            cur.end_idx=i; cur.num_points++;
        } else {
            /* visibility test using Heron's formula via sine */
            int64_t pr2 = (int64_t)pts[cur.end_idx].x*pts[cur.end_idx].x + (int64_t)pts[cur.end_idx].y*pts[cur.end_idx].y;
            int32_t pr = (int32_t)sqrt((double)pr2);
            int32_t a = r, b = pr, c = dist; /* triangle sides */
            /* Heron area: S = sqrt(p(p-a)(p-b)(p-c)), p=(a+b+c)/2 ; use double here, not in hot loops often */
            double p = (a + b + c) * 0.5;
            double S2 = p*(p-a)*(p-b)*(p-c);
            double S = S2>0 ? sqrt(S2) : 0.0;
            double sin_d = (a>0 && b>0) ? (2.0*S)/((double)a*(double)b) : 0.0;
            /* compare |sin_d| < sin(2*dp) */
            double sin_dp = ((double)P->sin_2dp_q15)/32768.0;
            if (fabs(sin_d) < fabs(sin_dp) && a < b) {
                cur.is_visible = 0;
            }

            if (cur.num_points >= P->min_group_points) {
                oe_push_pointset(out, cur);
            }
            cur.begin_idx=i; 
            cur.end_idx=i; 
            cur.num_points=1; 
            cur.is_visible = (fabs(sin_d) > fabs(sin_dp) || a < b);
        }
    }
    if (cur.num_points >= P->min_group_points) oe_push_pointset(out, cur);
}

/* ---------------- split & merge segmentation ---------------- */
void oe_detect_segments_rec(const OE_Point* pts, const OE_Params* P, OE_Output* out, OE_PointSet ps){
    if (ps.num_points < P->min_group_points) return;

    OE_Segment seg; seg_iepf_init(pts, ps.begin_idx, ps.end_idx, &seg);

    int32_t maxd = 0; int split = -1;
    for (int i=ps.begin_idx;i<=ps.end_idx;++i){
        int32_t d = oe_point_to_segment_dist(seg.first_point, seg.last_point, pts[i]);
        /* adaptive threshold: max_split_distance + range * proportion */
        int32_t r = (int32_t)sqrt((double)((int64_t)pts[i].x*pts[i].x + (int64_t)pts[i].y*pts[i].y));
        int32_t thr = P->max_split_distance + (int32_t)(((int64_t)r * P->distance_proportion)/1000000LL);
        if (d >= maxd && d > thr){ maxd = d; split = i; }
        }


        if (P->use_split_and_merge && split >= 0){
        int n1 = split - ps.begin_idx + 1;
        int n2 = ps.end_idx - split + 1;
        if (n1 > P->min_group_points && n2 > P->min_group_points){
            OE_PointSet ps1 = { ps.begin_idx, split, n1, ps.is_visible };
            OE_PointSet ps2 = { split, ps.end_idx, n2, ps.is_visible };
            oe_detect_segments_rec(pts, P, out, ps1);
            oe_detect_segments_rec(pts, P, out, ps2);
            return;
        }
    }
    seg.ps_begin = oe_push_pointset(out, ps); 
    seg.ps_count = (seg.ps_begin>=0)?1:0; 
    oe_push_segment(out, seg);
}


void oe_detect_segments(const OE_Point* pts, const OE_Params* P, OE_Output* out){
    int Nps = out->num_point_sets; 
    for(int i=0;i<Nps;++i) {
        oe_detect_segments_rec(pts, P, out, out->point_sets[i]);
    }
}

/* proximity test (similar to trueDistanceTo on endpoints) */
uint8_t oe_segments_prox(const OE_Segment* a, const OE_Segment* b, const OE_Params* P){
    int t = P->max_merge_separation;
    if (oe_point_to_segment_dist(a->first_point,a->last_point,b->first_point) < t) return 1;
    if (oe_point_to_segment_dist(a->first_point,a->last_point,b->last_point) < t) return 1;
    if (oe_point_to_segment_dist(b->first_point,b->last_point,a->first_point) < t) return 1;
    if (oe_point_to_segment_dist(b->first_point,b->last_point,a->last_point) < t) return 1;
    return 0;
}


/* collinearity: distance of each endpoint to merged best-fit line < spread */
uint8_t oe_segments_collinear(OE_Segment merged, const OE_Segment* s1, const OE_Segment* s2, const OE_Params* P){
    int m = P->max_merge_spread;
    if (oe_point_to_line_dist(merged.first_point, merged.last_point, s1->first_point) >= m) return 0;
    if (oe_point_to_line_dist(merged.first_point, merged.last_point, s1->last_point) >= m) return 0;
    if (oe_point_to_line_dist(merged.first_point, merged.last_point, s2->first_point) >= m) return 0;
    if (oe_point_to_line_dist(merged.first_point, merged.last_point, s2->last_point) >= m) return 0;
    return 1;
}


/* Fit segment over concatenated point sets by endpoints (IEPF style) */
OE_Segment oe_fit_segment_over_ps(const OE_Point* pts, const OE_PointSet* psets, int ps_begin, int ps_count){
    int b = psets[ps_begin].begin_idx;
    int e = psets[ps_begin + ps_count - 1].end_idx;
    OE_Segment s; 
    s.ps_begin = ps_begin; 
    s.ps_count = ps_count; 
    s.first_point = pts[b]; 
    s.last_point = pts[e];
    return s;
}


void oe_merge_segments(const OE_Point* pts, const OE_Params* P, OE_Output* out){
    for(int i=0;i<out->num_segments;++i){
        for(int j=i+1;j<out->num_segments;++j){
            OE_Segment *s1=&out->segments[i], *s2=&out->segments[j];
            /* enforce CCW by cross sign of first points vs origin (approx) */
            int64_t cross = (int64_t)s1->first_point.x * s2->first_point.y - (int64_t)s1->first_point.y * s2->first_point.x;
            if (cross < 0){ 
                OE_Segment* tmp=s1; s1=s2; s2=tmp; 
            }
            if (!oe_segments_prox(s1,s2,P)) continue;
            int base = out->num_point_sets;
            for(int k=0;k<s1->ps_count;++k){
                oe_push_pointset(out, out->point_sets[s1->ps_begin+k]);
            }
            for(int k=0;k<s2->ps_count;++k) {
                oe_push_pointset(out, out->point_sets[s2->ps_begin+k]);
            }
            int ps_cnt = s1->ps_count + s2->ps_count;
            OE_Segment merged = oe_fit_segment_over_ps(pts, out->point_sets, base, ps_cnt);
            if (oe_segments_collinear(merged, s1, s2, P)){
                out->segments[i] = merged;
                for(int m=j;m<out->num_segments-1;++m) {
                    out->segments[m]=out->segments[m+1];
                }
                out->num_segments--; j=i; /* recheck */
            }
        }
    }
}


void oe_detect_circles(const OE_Point* pts, const OE_Params* P, OE_Output* out){
    for(int si=0; si<out->num_segments; ++si){
        OE_Segment* s=&out->segments[si];
        if (P->circles_from_visibles){
            int vis=1; for(int k=0;k<s->ps_count;++k){ 
                if(!out->point_sets[s->ps_begin+k].is_visible){ 
                    vis=0; 
                    break; 
                } 
            }
            if (!vis) continue;
        }
        OE_Point c; int32_t r;
        if (!oe_fit_circle_kasa(pts, out->point_sets, s->ps_begin, s->ps_count, &c, &r)) continue;
        int32_t enlarged = r + P->radius_enlargement;
        if (enlarged < P->max_circle_radius){
            OE_Circle Cc = { c, enlarged, r, s->ps_begin, s->ps_count };
            oe_push_circle(out, Cc);
            if (P->discard_converted_segments){
                for(int m=si;m<out->num_segments-1;++m) {
                    out->segments[m]=out->segments[m+1];
                }
                out->num_segments--; si--; /* stay */
            }
        }
    }
}


uint8_t oe_compare_circles(const OE_Circle* a, const OE_Circle* b, const OE_Params* P, OE_Circle* out_merged){
    if (a==b) return 0;
    int64_t dx=(int64_t)b->center.x - a->center.x;
    int64_t dy=(int64_t)b->center.y - a->center.y;
    double dist = sqrt((double)(dx*dx + dy*dy));
    if ((b->radius - a->radius) >= dist){ 
        *out_merged = *b; return 1; 
    }
    if ((a->radius - b->radius) >= dist){ 
        *out_merged = *a; return 1; 
    }
    if ((a->radius + b->radius) >= dist){
        double t = (double)a->radius / (a->radius + b->radius);
        double cx = a->center.x + t * (double)dx;
        double cy = a->center.y + t * (double)dy;
        double rr = hypot((double)a->center.x - cx, (double)a->center.y - cy) + a->radius;
        int32_t R = (int32_t)llround(rr + (a->radius > b->radius ? a->radius : b->radius));
        if (R < P->max_circle_radius){ 
            *out_merged = (OE_Circle){ {(int32_t)llround(cx),(int32_t)llround(cy)}, R, (int32_t)llround(rr), 0, 0 }; 
            return 1; 
        }
    }
    return 0;
}

void oe_merge_circles(const OE_Params* P, OE_Output* out){
    for(int i=0;i<out->num_circles;++i){
        for(int j=i+1;j<out->num_circles;++j){
            OE_Circle m; 
            if (oe_compare_circles(&out->circles[i], &out->circles[j], P, &m)){
                out->circles[i]=m; 
                for(int k=j;k<out->num_circles-1;++k) {
                    out->circles[k]=out->circles[k+1]; 
                }
                out->num_circles--; j=i;
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
        OE_Point tc = apply_tf_q15(opt_tf, c.center);
        if (tc.x > P->min_x_limit && tc.x < P->max_x_limit && tc.y > P->min_y_limit && tc.y < P->max_y_limit){
            c.center = tc; 
            out->circles[wr++] = c;
        }
    }
    out->num_circles = wr;

    for (int i=0; i<out->num_segments; ++i){
        out->segments[i].first_point = apply_tf_q15(opt_tf, out->segments[i].first_point);
        out->segments[i].last_point = apply_tf_q15(opt_tf, out->segments[i].last_point);
    }
}


/* Precompute sin(2*dp) in Q15 from distance_proportion_ppm (approximate) */
void ensure_sin2dp(OE_Params* P){
    if (P->sin_2dp_q15!=0) return;
    float dp = (float)P->distance_proportion / 1e6f; /* unitless */
    float s = sinf(2.0f * dp);
    int32_t q = (int32_t)(s * 32768.0f);
    if (q==0) q = 1; /* avoid degenerate */
    P->sin_2dp_q15 = q;
}

void seg_iepf_init(const OE_Point* pts, int b, int e, OE_Segment* s){ 
    s->first_point=pts[b]; 
    s->last_point=pts[e]; 
}

