#ifndef _LIB_ASSERV_DEFAULT_H_
#define _LIB_ASSERV_DEFAULT_H_

/*############################################################################*/
/*                                    Odo                                     */
/*############################################################################*/

// Entraxe (distance roue-centre) en metres
#define DEFAULT_ODO_SPACING 0.131
#define DEFAULT_SIZE_WHEEL 0.060                      // 6cm (diametre)
#define DEFAULT_WHEEL_RADIUS (DEFAULT_SIZE_WHEEL / 2) // 3cm

/*############################################################################*/
/*                                  Motion                                    */
/*############################################################################*/
#define DEFAULT_CONSTRAINT_V_MAX 0.5f //1.5f
#define DEFAULT_CONSTRAINT_VT_MAX 6.0f

#define DEFAULT_CONSTRAINT_A_MAX 1.5f
#define DEFAULT_CONSTRAINT_AT_MAX 2.0f

/*############################################################################*/
/*                                  Asserv                                    */
/*############################################################################*/

#define ODO_EVERY_MS 1U
#define ASSERV_EVERY 10U

#define DEFAULT_STOP_DISTANCE 0.005 // +-5mm
#define DEFAULT_STOP_ANGLE 0.01745  // +-1deg  // en radian

#define DEFAULT_SPEED_LIN_STOP 0.05 // 5cm/s
#define DEFAULT_SPEED_ROT_STOP 0.05 // 0.05 rad/s (~2.9 deg/s)

/*############################################################################*/
/*                                   PID                                      */
/*############################################################################*/
// PID dre la vitesse de chaque roue
#define DEFAULT_PID_V_LIN_KP 8000 // kp
#define DEFAULT_PID_V_LIN_KI 20   // ki
#define DEFAULT_PID_V_LIN_KD 0    // kd

/*############################################################################*/
/*                                   MOTOR                                    */
/*############################################################################*/
#define MOTOR_ANGLE_CODEUR_MAX 8192

#define MOTOR_POWER_MAX 10000
#define MOTOR_POWER_MIN -10000

#define ODO_MOTOR_REDUCTION 36.0f   // à ajuster si différent

#define ODO_MOTOR_1_SIGN  1.0f
#define ODO_MOTOR_2_SIGN  1.0f
#define ODO_MOTOR_3_SIGN  1.0f
#define ODO_MOTOR_4_SIGN  1.0f

#endif // _LIB_ASSERV_DEFAULT_H_
