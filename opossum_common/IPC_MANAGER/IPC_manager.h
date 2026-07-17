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

#endif // IPC_MANAGER_H
