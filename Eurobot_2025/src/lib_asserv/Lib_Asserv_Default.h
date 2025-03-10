#ifndef _LIB_ASSERV_DEFAULT_H_
#define _LIB_ASSERV_DEFAULT_H_

/*############################################################################*/
/*                                    Odo                                     */
/*############################################################################*/

// {tic/m, m/tic, entre roues}
#define DEFAULT_ODO_SPACING  0.115
#define DEFAULT_SIZE_WHEEL 0.0575 // 6cm
#define DEFAULT_WHEEL_RADIUS DEFAULT_SIZE_WHEEL/2 // 3cm

/*############################################################################*/
/*                                  Motion                                    */
/*############################################################################*/

// {v, vt} v = 0.9 * v max moteur, vt = v/(entre roues/2)
#define DEFAULT_CONSTRAINT_V_MAX  0.5//1.0
#define DEFAULT_CONSTRAINT_VT_MAX 0.5

#define DEFAULT_AUTHORIZED_V_MAX 0.5
#define DEFAULT_AUTHORIZED_VT_MAX 0.5

#define DEFAULT_CONSTRAINT_V_ROUE_MAX 1


/* {a, at, v_vt} a = a max sans glissement, at = a/(entre roues/2),
 * v_vt = acc centripete (trop fort -> erreur odo) 0.1g
 *
 */
#define DEFAULT_CONSTRAINT_A_MAX 0.25
#define DEFAULT_CONSTRAINT_AT_MAX 0.5

//#define DEFAULT_CONSTRAINT_VVT_MAX 0.981

#define DEFAULT_CONSTRAINT_A_ROUE 0.5

#define ASSERV_BLOCK_TIME_LIMIT 1   // 1s "blocké" avant de tout couper

/*############################################################################*/
/*                                  Asserv                                    */
/*############################################################################*/

#define DEFAULT_STOP_DISTANCE 0.005 // +-5mm
#define DEFAULT_STOP_ANGLE 0.01745// +-1deg  // en radian

/*############################################################################*/
/*                                   PID                                      */
/*############################################################################*/

// 2 PID lies a l'asserv en position (dist: position absolue, angle: position angulaire)
#define DEFAULT_PID_DIST_KP 3 // kp   
#define DEFAULT_PID_DIST_KI 0 //ki    
#define DEFAULT_PID_DIST_KD 0  //kd     

#define DEFAULT_PID_ANGLE_KP 3 // kp
#define DEFAULT_PID_ANGLE_KI 0 //ki
#define DEFAULT_PID_ANGLE_KD 0 //kd    


// 2 PID lies a l'asserve en vitesse (vitesse lineaire et vitesse angulaire)
#define DEFAULT_PID_V_LIN_KP 10000 // kp
#define DEFAULT_PID_V_LIN_KI 100   //ki
#define DEFAULT_PID_V_LIN_KD 0   //kd   

#endif // _LIB_ASSERV_DEFAULT_H_

