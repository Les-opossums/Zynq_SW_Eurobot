#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "shared_memory_structure.h"
#define SHARED_MEMORY_BASEADDR 0xFFFF0000 // Base address for shared memory

#define SHARE_MEM_SGI_INT_ID    14          // ID de l'interruption logicielle (SGI)
#define SGI_TARGET_CORE_1       0x00000002  // Bit 1 = Cibler le Core 1 (0x1 = Core 0, 0x2 = Core 1)

extern volatile sharedCommand *shared_mem;
extern sharedCommand local_data;

#define SEND_FIELD(data_ptr, field_name) \
    send_to_other_core(\
        &(data_ptr)->field_name, \
        sizeof((data_ptr)->field_name), \
        &(shared_mem)->field_name, \
        &(shared_mem)->flag_##field_name##_valid, \
        &(shared_mem)->flag_##field_name##_ack)

#define SEND_FIELD_BLOCKING(data_ptr, field_name) \
    send_to_other_core_blocking( \
        &(data_ptr)->field_name, \
        sizeof((data_ptr)->field_name), \
        &(shared_mem)->field_name, \
        &(shared_mem)->flag_##field_name##_valid, \
        &(shared_mem)->flag_##field_name##_ack)

#define CHECK_FIELD(data_ptr, field_name) \
    check_from_other_core( \
        &(data_ptr)->field_name, \
        sizeof((shared_mem)->field_name), \
        &(shared_mem)->field_name, \
        &(shared_mem)->flag_##field_name##_valid, \
        &(shared_mem)->flag_##field_name##_ack)

/**
 * @brief Initialize the shared memory area
 * 
 */
void init_shared_memory(void); 

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
int send_to_other_core(const void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack);

/**
 * @brief This function write data to the shared memory area only if ha been readby the other core
 * 
 * @param data pointer to the data to send 
 * @param size size of the data to send
 * @param dest pointer to the destination in shared memory
 * @param flag_valid pointer to the flag indicating if the data is valid
 * @param flag_ack pointer to the flag indicating if the data has been acknowledged
 */
void send_to_other_core_blocking(const void *data, size_t size,
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
int check_from_other_core(void *data_out, size_t size,
                          const volatile void *src,
                          volatile uint32_t *flag_valid,
                          volatile uint32_t *flag_ack);



/**
 * @brief This function triggers an interrupt on the other core to signal that new data is available in shared memory
 * 
 */
void trigger_core1_interrupt(void);
#endif // SHARED_MEMORY_H
