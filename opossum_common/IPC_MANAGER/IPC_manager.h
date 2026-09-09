#ifndef IPC_MANAGER_H
#define IPC_MANAGER_H

#include "../IPC_structure.h"
#include "stddef.h"


#define IPC_SHARED_MEM_ADDR 0xFFFF0000 // OCM
#define IPC_SGI_INT_ID      14          // ID de l'interruption logicielle (SGI)

#define IPC_DATA ((ipc_shared_data_t *)IPC_SHARED_MEM_ADDR) // Pointeur vers la structure de données partagées  


#define SEND_FIELD(data_ptr, field_name) \
    IPC_SendToOtherCore(\
        &(data_ptr)->field_name, \
        sizeof((data_ptr)->field_name), \
        &(IPC_DATA)->field_name, \
        &(IPC_DATA)->flag_##field_name##_valid, \
        &(IPC_DATA)->flag_##field_name##_ack)

// Variante FIABLE de SEND_FIELD pour les commandes one-shot : si le canal est
// encore occupe (message precedent non consomme), la valeur est memorisee et
// re-emise automatiquement par IPC_RetryPending() (appele dans la boucle
// principale) jusqu'a passer -- sans jamais bloquer. A reserver aux commandes
// (position, mode, consignes) ; la telemetrie periodique reste en SEND_FIELD.
#define SEND_FIELD_RELIABLE(data_ptr, field_name) \
    IPC_SendReliable( \
        &(IPC_DATA)->field_name, \
        &(IPC_DATA)->flag_##field_name##_valid, \
        &(IPC_DATA)->flag_##field_name##_ack, \
        &(data_ptr)->field_name, \
        sizeof((data_ptr)->field_name))


#define CHECK_FIELD(data_ptr, field_name) \
    IPC_CheckFromOtherCore( \
        &(data_ptr)->field_name, \
        sizeof((IPC_DATA)->field_name), \
        &(IPC_DATA)->field_name, \
        &(IPC_DATA)->flag_##field_name##_valid, \
        &(IPC_DATA)->flag_##field_name##_ack)

/**
 * @brief Initialize the shared memory area
 * 
 */
void IPC_Init(void); 

void IPC_SyncCores(void);

/**
 * @brief This function write data to the shared memory area
 * 
 * @param data pointer to the data to send 
 * @param size size of the data to send
 * @param dest pointer to the destination in shared memory
 * @param flag_valid pointer to the flag indicating if the data is valid
 * @param flag_ack pointer to the flag indicating if the data has been acknowledged
 * 
 * @return int 1 if data sent, 0 if not sent (previous data not acknowledged)
 */
int IPC_SendToOtherCore(const void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack);

/**
 * @brief This function checks if there is data from the other core
 * 
 * @param data_out pointer to the data to receive
 * @param size size of the data to receive
 * @param src pointer to the source in shared memory
 * @param flag_valid pointer to the flag indicating if the data is valid
 * @param flag_ack pointer to the flag indicating if the data has been acknowledged
 * @return int 1 if data received, 0 if nothing to read
 */
int IPC_CheckFromOtherCore(void *data_out, size_t size,
                           const volatile void *src,
                           volatile uint32_t *flag_valid,
                           volatile uint32_t *flag_ack);

/* ===================================================================== *
 * Envoi FIABLE non bloquant (retransmission automatique)
 * ===================================================================== */
#define IPC_RELIABLE_MAX_SLOTS  16   /* nb de canaux one-shot suivis         */
#define IPC_RELIABLE_MAX_SIZE   32   /* >= taille du plus gros payload (o)   */

/**
 * @brief Envoi FIABLE non bloquant. Memorise la DERNIERE valeur voulue pour le
 * canal 'dest' et tente de l'emettre. Si le canal est occupe, la valeur reste
 * en attente et sera re-emise par IPC_RetryPending(). Un canal = un slot (la
 * derniere valeur ecrase la precedente : c'est bien l'etat voulu qui part).
 * @return 1 si emis immediatement, 0 si mis en attente, -1 si payload trop gros.
 */
int  IPC_SendReliable(volatile void *dest, volatile uint32_t *flag_valid,
                      volatile uint32_t *flag_ack, const void *data, size_t size);

/**
 * @brief Re-emet toutes les commandes fiables encore en attente. A appeler a
 * chaque tour de la boucle principale (les deux coeurs).
 */
void IPC_RetryPending(void);

#endif // IPC_MANAGER_H
