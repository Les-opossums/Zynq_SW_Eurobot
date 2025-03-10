
#include "lib_asserv.h"


// Renvoyer une valeur comprise entre inf et sup

float limit_float(float valeur, float inf, float sup) {
    if (valeur < inf) return inf;
    else if (valeur > sup) return sup;
    else return valeur;
}

// angle principal

float principal_angle(float angle) {
    float alpha;
    if (fabsf(angle) < PI) {
        return angle;
    } else {
        alpha = fmodf(angle, 2 * PI);
        if (alpha<-PI) {
            alpha += 2 * PI;
        } // d�calage de 2PI remarqu� par Ken si l'angle est n�gatif
        if (alpha > PI) {
            alpha -= 2 * PI;
        }
        return alpha;
    }
}
