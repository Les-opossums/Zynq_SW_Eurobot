#ifndef FEETECH_H
#define FEETECH_H

#define FEETECH_PROTOCOL_DEBUG

typedef unsigned char byte;
// Broadcast ID

#define FEETECH_DIR_GPIO_BASEADDR XPAR_AXI_GPIO_24_BASEADDR
#define FEETECH_DIR_CHANNEL 1  // canal 1 du GPIO
#define FEETECH_DIR_TX 1
#define FEETECH_DIR_RX 0

#define FEETECH_H

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

//-------SRAM(只读)--------
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
    uint16_t Uart_Brg;  // vitesse 
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


void Init_Com_FEETECH(void);

void FEETECH_Loop(void);

void FEETECH_Cmd_Send(FEETECH_Command *Cmd);

uint8_t RegisterLenFEETECH(uint8_t address);

void Add_FEETECH_Cmd(uint8_t FEETECH_Addr, uint16_t Uart_Brg, uint8_t Command, uint8_t Reg_Addr, uint32_t Data_To_Send, void *Data_Answer, uint8_t Nb_Data, uint8_t *Status, void *Done, uint8_t Protocol);

void PutFEETECH(uint8_t id, uint8_t Reg, uint32_t Data);
void PutFEETECH_Wait(uint8_t id, uint8_t Reg, uint32_t Data); // interdit dans le robot, sauf pour debug !!!
void PutFEETECH_Ext_Done(uint8_t id, uint8_t Reg, uint32_t Data, void *Done);

void GetFEETECH(uint8_t id, uint8_t Reg, void *Data_Answer);
uint32_t GetFEETECH_Wait(uint8_t id, uint8_t Reg); // interdit dans le robot, sauf pour debug !!!
void GetFEETECH_Ext_Done(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done);

uint8_t FEETECH_All_Cmd_Done(void);

void FEETECH_Uart_EventHandler(unsigned int Event, unsigned int EventData);

void GetFEETECH_Ext_Done_With_Status(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done, uint8_t *Status);

// void usleep(uint32_t usec);

void PutFEETECH_SCS(uint8_t id, uint8_t Reg, uint32_t Data);
void PutFEETECH_Wait_SCS(uint8_t id, uint8_t Reg, uint32_t Data);
void PutFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, uint32_t Data, void *Done);
uint32_t GetFEETECH_Wait_SCS(uint8_t id, uint8_t Reg);
void GetFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done);
#endif // FEETECH_H