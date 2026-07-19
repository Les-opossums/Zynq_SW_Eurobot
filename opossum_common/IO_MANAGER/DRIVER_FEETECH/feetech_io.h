#ifndef DRIVER_FEETECH_IO_H
#define DRIVER_FEETECH_IO_H

#include "xil_types.h"
#include "xgpio.h"
#include "../DRIVER_UART_PS/driver_uart_ps.h"

/* ─── Debug ──────────────────────────────────────────────────────────────
 * Decommenter pour reactiver les prints de diagnostic du protocole FEETECH
 * (timeouts, erreurs de checksum, retries...). Desactive par defaut : le
 * rapport d'init unique de IO_Manager (cf main.c) suffit en usage normal.
 */
// #define FEETECH_DEBUG

#if defined(FEETECH_DEBUG)
#include "xil_printf.h"
#define FEETECH_LOG(...) xil_printf(__VA_ARGS__)
#else
#define FEETECH_LOG(...) do {} while (0)
#endif

typedef unsigned char byte;

/* Niveaux logiques du GPIO de direction du bus half-duplex (1 = emission,
 * 0 = reception). A adapter si le sens du buffer/mux materiel est inverse. */
#define FEETECH_DIR_TX 1
#define FEETECH_DIR_RX 0

/* Adressage bus FEETECH */
#define FEETECH_BROADCAST               254

#define FEETECH_CMD_NB_MAX_TRY_SEND 3
#define FEETECH_CMD_LIST_SIZE 1000
#define FEETECH_CMD_BUFF_LENGTH 40

#define FEETECH_STATUS_OK              0
#define FEETECH_STATUS_UNSUPORTED_CMD  0x81
#define FEETECH_STATUS_TIMEOUT         0x82
#define FEETECH_STATUS_CHKSUM_ERROR    0x83

// Instruction Set
#define FEETECH_INST_PING                1
#define FEETECH_INST_READ_DATA           2
#define FEETECH_INST_WRITE_DATA          3
#define FEETECH_INST_REG_WRITE           4
#define FEETECH_INST_ACTION              5
#define FEETECH_INST_RESET               6
#define FEETECH_INST_SYNC_WRITE          131

//-------EPROM--------
#define  FEETECH_MODEL_L 3
#define  FEETECH_MODEL_H 4

//-------EPROM--------
#define  FEETECH_ID 5
#define  FEETECH_BAUD_RATE 6
#define  FEETECH_DELAY_TIME_RETURN 7
#define  FEETECH_LEVEL_RETURN 8
#define  FEETECH_MIN_ANGLE_LIMIT_L 9
#define  FEETECH_MIN_ANGLE_LIMIT_H 10
#define  FEETECH_MAX_ANGLE_LIMIT_L 11
#define  FEETECH_MAX_ANGLE_LIMIT_H 12
#define  FEETECH_MAX_TEMP_LIMIT 13
#define  FEETECH_MAX_INPUT_VOLT 14
#define  FEETECH_MIN_INPUT_VOLT 15
#define  FEETECH_MAX_TORQUE_LIMIT_L 16
#define  FEETECH_MAX_TORQUE_LIMIT_H 17
#define  FEETECH_SETTING_BYTE 18
#define  FEETECH_PROTECTION_ENABLE 19
#define  FEETECH_ALARM_LED 20
#define  FEETECH_MIN_START_TORQUE 24
#define  FEETECH_CW_DEAD 26
#define  FEETECH_CCW_DEAD 27
#define  FEETECH_OVERLOAD_CURRENT_L 28
#define  FEETECH_OVERLOAD_CURRENT_H 29
#define  FEETECH_RESOLUTION 30
#define  FEETECH_OFS_L 31
#define  FEETECH_OFS_H 32
#define  FEETECH_MODE 33

//-------SRAM--------
#define  FEETECH_TORQUE_ENABLE 40
#define  FEETECH_ACC 41
#define  FEETECH_GOAL_POSITION_L 42
#define  FEETECH_GOAL_POSITION_H 43
#define  FEETECH_GOAL_TIME_L 44
#define  FEETECH_GOAL_TIME_H 45
#define  FEETECH_GOAL_SPEED_L 46
#define  FEETECH_GOAL_SPEED_H 47
#define  FEETECH_TORQUE_LIMIT_L 48
#define  FEETECH_TORQUE_LIMIT_H 49
#define  FEETECH_LOCK 55

//-------SRAM (lecture seule)--------
#define  FEETECH_PRESENT_POSITION_L 56
#define  FEETECH_PRESENT_POSITION_H 57
#define  FEETECH_PRESENT_SPEED_L 58
#define  FEETECH_PRESENT_SPEED_H 59
#define  FEETECH_PRESENT_LOAD_L 60
#define  FEETECH_PRESENT_LOAD_H 61
#define  FEETECH_PRESENT_VOLTAGE 62
#define  FEETECH_PRESENT_TEMPERATURE 63
#define  FEETECH_MOVING 66
#define  FEETECH_PRESENT_CURRENT_L 69
#define  FEETECH_PRESENT_CURRENT_H 70

#define PUMP_CMD_1 80
#define PUMP_CMD_2 81
#define VALVE_CMD_1 82
#define VALVE_CMD_2 83

#define ADDR_CURRENT_1_L 84
#define ADDR_CURRENT_1_H 85
#define ADDR_CURRENT_2_L 86
#define ADDR_CURRENT_2_H 87

typedef struct {
    uint16_t Uart_Brg;  // vitesse (non utilise pour l'instant : baud fixe, cf FEETECH_IO_Init)
    uint8_t FEETECH_Addr;
    uint8_t Command;
    uint8_t Reg_Addr;
    uint32_t Data_To_Send;
    void *Data_Answer;
    uint8_t Nb_Data; // Max 4 en send
    uint8_t *Status;
    void *Done;
    uint8_t Protocol;
} FEETECH_Command;

/* --- Contexte de l'instance (cf IO_config.h / IO_globals.c) ---
 * Le transport bas niveau (UART1) est un device IO_Manager a part entiere
 * (driver generique DRIVER_UART_PS, cf UartFeetech_Ctx), reference ici par
 * pointeur. Ce contexte n'ajoute que ce qui est specifique au protocole
 * FEETECH : le pilotage de la broche de direction du bus half-duplex.
 */
typedef struct {
    uart_ps_context_t *uart;    // Transport UART1 (deja initialise par son propre device IO_Manager)

    u32 dir_gpio_device_id;     // XPAR_AXI_GPIO_x_DEVICE_ID de l'AXI GPIO de direction
    u32 dir_gpio_channel;       // Canal du GPIO pilotant le buffer half-duplex
    XGpio dir_gpio;             // Instance Xilinx (remplie par FEETECH_IO_Init)
} feetech_io_context_t;

/* --- Prototypes standards pour l'IO_Manager --- */
int  FEETECH_IO_Init(void *instance);
void FEETECH_IO_Update(void *instance);

/* --- API publique (identique a l'ancienne API feetech.c/.h) --- */
uint8_t RegisterLenFEETECH(uint8_t address);

void Add_FEETECH_Cmd(uint8_t FEETECH_Addr, uint16_t Uart_Brg, uint8_t Command, uint8_t Reg_Addr, uint32_t Data_To_Send, void *Data_Answer, uint8_t Nb_Data, uint8_t *Status, void *Done, uint8_t Protocol);

// ---- STS (protocole "petit-endian", servos type STS3215) ----
void PutFEETECH(uint8_t id, uint8_t Reg, uint32_t Data);
void PutFEETECH_Wait(uint8_t id, uint8_t Reg, uint32_t Data); // interdit dans le robot, sauf pour debug !!!
void PutFEETECH_Ext_Done(uint8_t id, uint8_t Reg, uint32_t Data, void *Done);

void GetFEETECH(uint8_t id, uint8_t Reg, void *Data_Answer);
uint32_t GetFEETECH_Wait(uint8_t id, uint8_t Reg); // interdit dans le robot, sauf pour debug !!!
void GetFEETECH_Ext_Done(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done);
void GetFEETECH_Ext_Done_With_Status(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done, uint8_t *Status);

// ---- SCS (protocole "grand-endian" sur 2 octets, servos type SCS0009) ----
void PutFEETECH_SCS(uint8_t id, uint8_t Reg, uint32_t Data);
void PutFEETECH_Wait_SCS(uint8_t id, uint8_t Reg, uint32_t Data);
void PutFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, uint32_t Data, void *Done);
uint32_t GetFEETECH_Wait_SCS(uint8_t id, uint8_t Reg);
void GetFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done);

uint8_t FEETECH_All_Cmd_Done(void);

#endif // DRIVER_FEETECH_IO_H
