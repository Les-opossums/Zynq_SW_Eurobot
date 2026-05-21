#ifndef __KALMAN_FIFO_H_
#define __KALMAN_FIFO_H_

#include "kalman.h"
#include "Asserv_type.h"

#define KALMAN_FIFO_LEN 200
typedef struct {
    uint8_t has_lidar;        // Flag : y a-t-il eu un lidar à cet instant ?
    float z_lidar[3];         // La mesure lidar

    uint8_t has_camera[3];       // Flag : y a-t-il eu une caméra à cet instant ?
    float z_camera[3][3];        // La mesure caméra
    float r_camera[3][3];        // Le bruit de la caméra

    uint8_t bypass_lidar_rejection;
    uint8_t bypass_camera_rejection[3];
} Observations;

typedef struct {
    KalmanState buffer[KALMAN_FIFO_LEN];
    Speed speed_robot[KALMAN_FIFO_LEN];
    Observations observations[KALMAN_FIFO_LEN];
    int head; // index de la tête de la FIFO
    int count; // nombre d'éléments dans la FIFO actuellement valides
} KalmanFIFO;

extern KalmanFIFO kalman_fifo;

/**
 * Initialise la FIFO (position tête à 0, mémoire à zéro).
 */
void kalman_fifo_init(KalmanFIFO* fifo);

/**
 * Ajoute un état à la FIFO.
 * 
 * @param fifo La structure FIFO.
 * @param state L'état à ajouter.
 * @param speed_robot La vitesse du robot à ajouter (pour la prédiction).
 */
void kalman_fifo_push(KalmanFIFO* fifo, KalmanState* state, Speed* speed_robot);

/**
 * Récupère un état dans le passé à un délai donné (en ms).
 * 
 * @param fifo La structure FIFO.
 * @param delay_ms Le délai souhaité (ex : 100 ms).
 * @param dt_ms Période de l’odométrie en ms (ex : 1.0f).
 * 
 * @return L'état à l'index correspondant au délai, ou NULL si le délai est trop long.
 */
int kalman_fifo_get_delay(KalmanFIFO* fifo, int delay_ms, float dt_ms);


/**
 * Repropagation des états dans la FIFO à partir d’un état corrigé.
 *
 * @param fifo La structure FIFO.
 * @param delay_index L’index de l’état corrigé dans la FIFO.
 * @param dt_s Le pas de temps (s).
 * @param R_lidar Les profils de bruit lidar.
 */
void kalman_fifo_repropagate(KalmanFIFO* fifo, int delay_index, float dt_s, float R_lidar[3]);

/**
 * @brief Initialise la kalman avec les valeurs du lidar
 * 
 * @param fifo fifo de kalman
 * @param pos lidar_pos
 */
void kalman_init_with_lidar(KalmanFIFO* fifo, Position* lidar_pos);


#endif // __KALMAN_FIFO_H_