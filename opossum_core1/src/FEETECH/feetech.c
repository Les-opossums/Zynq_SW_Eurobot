#include "../main.h" // keep original defines and FEETECH_* constants

// #define DEBUG

#define COM_FEETECH_IDDLE           0x00
#define COM_FEETECH_SENDING         0x01
#define COM_FEETECH_WAIT_ANSWER     0x02

#define FEETECH_PROTO_STS           0 // Little Endian (STS3215...)
#define FEETECH_PROTO_SCS           1 // Big Endian (SCS0009...)

extern XUartPs Uart1_Instance; // from your uart code (you had this variable)

#define RX_TIMEOUT_MS_DEFAULT 20

uint8_t Com_FEETECH_Status = COM_FEETECH_IDDLE;
uint32_t Time_Of_Last_FEETECH_Received = 0;
uint32_t FEETECH_IFG_Timer = 0;
uint32_t Com_FEETECH_Maxtime = 10;

uint8_t FEETECH_Loop_State = 0;

uint8_t FEETECH_Transmit_Tab[FEETECH_CMD_BUFF_LENGTH] = {0xFF, 0xFF, 0};
uint8_t FEETECH_Transmit_Goal = 0, FEETECH_Transmit_Ptr = 0;
uint8_t FEETECH_Receive_Tab[FEETECH_CMD_BUFF_LENGTH];
uint8_t FEETECH_Receive_Ptr = 0;

uint8_t FEETECH_Bytes_To_Ignore = 0;

FEETECH_Command Liste_Command_FEETECH[FEETECH_CMD_LIST_SIZE];
uint8_t Command_FEETECH_TODO = 0;
uint8_t Command_FEETECH_DONE = 0;
uint8_t FEETECH_Cmd_Nb_Try = 0;
uint8_t FEETECH_Dumy = 0;

/* keep small local copies like before */
uint8_t feetech_torque_enable;
uint8_t feetech_id ;
uint8_t feetech_goal_position;
uint8_t feetech_moving_speed;
uint8_t feetech_torque_limit;
uint8_t feetech_present_position;
uint8_t feetech_present_speed;
uint32_t BRGVALFEETECH;

volatile uint8_t feetech_ignore_echo = 0;

/* internal flags */
static volatile uint8_t feetech_tx_done = 0; /* set by FEETECH_Uart_EventHandler when SENT event occurs */

/* forward */
uint8_t RegisterLenFEETECH(uint8_t address);

static XGpio GpioFeetechDir;  // instance AXI GPIO

/* Initialize FEETECH port using board-specific macros */
void Init_Com_FEETECH(void){
    /* Put bus in receive mode */
    XGpio_Initialize(&GpioFeetechDir, XPAR_AXI_GPIO_6_DEVICE_ID);
    XGpio_SetDataDirection(&GpioFeetechDir, FEETECH_DIR_CHANNEL, 0x0); // en sortie
    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_RX);

    feetech_torque_enable = FEETECH_TORQUE_ENABLE;
    feetech_id = FEETECH_ID;
    feetech_goal_position = FEETECH_GOAL_POSITION_L;
    feetech_moving_speed = FEETECH_GOAL_SPEED_L;
    feetech_torque_limit = FEETECH_TORQUE_LIMIT_L;
    feetech_present_position = FEETECH_PRESENT_POSITION_L;
    feetech_present_speed = FEETECH_PRESENT_SPEED_L;
}

/* This function must be called by your UART1 handler to forward events */
void FEETECH_Uart_EventHandler(unsigned int Event, unsigned int EventData)
{
    if (Event == XUARTPS_EVENT_SENT_DATA) {
        feetech_tx_done = 1;
    } else if (Event == XUARTPS_EVENT_RECV_DATA || Event == XUARTPS_EVENT_RECV_TOUT) {
        /* we don't process bytes here; FEETECH_Loop will fetch bytes via Get_Uart1_Cmd */
        Time_Of_Last_FEETECH_Received = Timer_ms1;
    } else if (Event == XUARTPS_EVENT_RECV_ERROR || Event == XUARTPS_EVENT_PARE_FRAME_BRKE ||
               Event == XUARTPS_EVENT_RECV_ORERR ) {
        /* track time so FEETECH_Loop's timeout logic works */
        Time_Of_Last_FEETECH_Received = Timer_ms1;
    }
}

// void usleep(uint32_t usec) {
//     volatile uint32_t wait;
//     uint32_t iterations = (usec * 667) / 10; // assuming 667 MHz clock, adjust as needed
//     for (wait = 0; wait < iterations; wait++){
//         __asm__ __volatile__ ("");
//     }
// }

/* Send a prepared command buffer using your XUartPs wrapper */
void FEETECH_Cmd_Send(FEETECH_Command *Cmd) {
    uint8_t i;
    unsigned int checksum_sum = 0;
    
    FEETECH_Transmit_Tab [0] = 0xFF;
    FEETECH_Transmit_Tab [1] = 0xFF;
    FEETECH_Transmit_Tab [2] = Cmd->FEETECH_Addr;
    checksum_sum += Cmd->FEETECH_Addr;

    if(Cmd->Command == FEETECH_INST_WRITE_DATA) {
        FEETECH_Transmit_Tab [4] = Cmd->Command;
        checksum_sum += Cmd->Command;
        FEETECH_Transmit_Tab [5] = Cmd->Reg_Addr;
        checksum_sum += Cmd->Reg_Addr;
        
        uint32_t Data_To_Send = Cmd->Data_To_Send;
        
        // --- MODIFICATION ICI : GESTION ENDIANNESS ---
        if (Cmd->Protocol == FEETECH_PROTO_SCS && Cmd->Nb_Data == 2) {
            // SCS (Big Endian) : On envoie d'abord le poids FORT (MSB)
            // Send High Byte
            FEETECH_Transmit_Tab [6] = (uint8_t)((Data_To_Send >> 8) & 0xFF);
            checksum_sum += FEETECH_Transmit_Tab [6];
            // Send Low Byte
            FEETECH_Transmit_Tab [7] = (uint8_t)(Data_To_Send & 0xFF);
            checksum_sum += FEETECH_Transmit_Tab [7];
        } 
        else {
            // STS (Little Endian) OU SCS 1 byte : Standard (LSB First)
            for (i = 0; i < Cmd->Nb_Data; i++) {
                FEETECH_Transmit_Tab [6 + i] = (uint8_t)(Data_To_Send & 0xFF);
                checksum_sum += (uint8_t)(Data_To_Send & 0xFF);
                Data_To_Send = (Data_To_Send >> 8);
            }
        }
        // ---------------------------------------------

        FEETECH_Transmit_Tab [3] = Cmd->Nb_Data + 3; // Len
        checksum_sum += Cmd->Nb_Data + 3;

    } else if(Cmd->Command == FEETECH_INST_READ_DATA) {
        FEETECH_Transmit_Tab [4] = FEETECH_INST_READ_DATA;
        checksum_sum += FEETECH_INST_READ_DATA;

        FEETECH_Transmit_Tab [5] = Cmd->Reg_Addr;
        checksum_sum += Cmd->Reg_Addr;

        FEETECH_Transmit_Tab [6] = Cmd->Nb_Data;
        checksum_sum += Cmd->Nb_Data;

        FEETECH_Transmit_Tab [3] = 4; 
        checksum_sum += 4;
    }

    uint8_t calculate_chk = (uint8_t)(~checksum_sum);
    FEETECH_Transmit_Goal = (uint8_t)(FEETECH_Transmit_Tab [3] + 4); 
    FEETECH_Transmit_Tab[FEETECH_Transmit_Goal - 1] = calculate_chk;

    // usleep(2000);
    UART1_Flush_RX();

    FEETECH_Transmit_Ptr = 0;
    FEETECH_Receive_Ptr = 0;
    FEETECH_Bytes_To_Ignore = FEETECH_Transmit_Goal; 

    Xil_DCacheFlushRange((UINTPTR)FEETECH_Transmit_Tab, FEETECH_Transmit_Goal);
    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_TX);
    feetech_tx_done = 0;

    Send_Uart1_Buff_Cmd(FEETECH_Transmit_Tab, FEETECH_Transmit_Goal);

    Com_FEETECH_Status = COM_FEETECH_SENDING;
    Time_Of_Last_FEETECH_Received = Timer_ms1;
    Com_FEETECH_Maxtime = RX_TIMEOUT_MS_DEFAULT;
}
/* The original FEETECH loop adapted to pull RX bytes from your RX ring using Get_Uart1_Cmd */
void FEETECH_Loop(void){
    uint8_t val8, i;
    uint8_t b;

    // Récupération des octets UART (inchangé)
    while (Get_Uart1_Cmd(&b)) {
        if (feetech_ignore_echo == 0){
            if (FEETECH_Bytes_To_Ignore > 0) {
                FEETECH_Bytes_To_Ignore--;
                continue; 
            }
            if(FEETECH_Receive_Ptr == 0){
                if(b == 0xFF){
                    FEETECH_Receive_Tab[FEETECH_Receive_Ptr++] = b;
                    Time_Of_Last_FEETECH_Received = Timer_ms1;
                }
            }
            else if (FEETECH_Receive_Ptr == 1){
                if(b == 0xFF){
                    FEETECH_Receive_Tab[FEETECH_Receive_Ptr++] = b;
                    Time_Of_Last_FEETECH_Received = Timer_ms1;
                } else {
                    FEETECH_Receive_Ptr = 0;
                }
            }
            else {
                FEETECH_Receive_Tab[FEETECH_Receive_Ptr] = b;
                Time_Of_Last_FEETECH_Received = Timer_ms1;
                if(FEETECH_Receive_Ptr < (FEETECH_CMD_BUFF_LENGTH - 1)){
                    FEETECH_Receive_Ptr++;
                } else {
                    FEETECH_Receive_Ptr = 0;
                }
            }
        }
    }

    switch(FEETECH_Loop_State) {
        case 0:
            if (Command_FEETECH_TODO != Command_FEETECH_DONE){
                FEETECH_Loop_State++;
            }
            break;

        case 1:
            FEETECH_Cmd_Nb_Try = 0;
            // Vérification de validité basique
            if (((Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_READ_DATA) &&
                 (Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr != FEETECH_BROADCAST)) ||
                 (Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_WRITE_DATA)){
                FEETECH_Loop_State = 10;
            } else {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_UNSUPORTED_CMD;
                FEETECH_Loop_State = 100;
            }
            break;

        case 10:
            FEETECH_Cmd_Send(&Liste_Command_FEETECH[Command_FEETECH_DONE]);
            FEETECH_Loop_State++;
            break;
        case 11:
            if (feetech_tx_done) {
                if (XUartPs_IsTransmitEmpty(&Uart1_Instance)) {
                    usleep(150); 
                    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_RX);
                    feetech_tx_done = 0;
                    FEETECH_Receive_Ptr = 0;
                    Time_Of_Last_FEETECH_Received = Timer_ms1;
                    FEETECH_Loop_State = 20; 
                }
            }
            break;

        case 20:
            Com_FEETECH_Status = COM_FEETECH_WAIT_ANSWER;
            Time_Of_Last_FEETECH_Received = Timer_ms1;
            FEETECH_Loop_State = 21;
            break;
            
        case 21:
            if (Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr == FEETECH_BROADCAST) {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_OK;
                FEETECH_Loop_State = 100;
            } else if ((FEETECH_Receive_Ptr > 3)){
                if((FEETECH_Receive_Tab[3] == (FEETECH_Receive_Ptr - 4)) ) {
                    FEETECH_Loop_State = 30;
                }else if((Timer_ms1 - Time_Of_Last_FEETECH_Received) > Com_FEETECH_Maxtime){
                    #ifdef FEETECH_PROTOCOL_DEBUG
                         // --- PRINT TIMEOUT ---
                        printf("FEETECH Error: Timeout (Partial Rx) on ID %d (Reg 0x%02X)\r\n", 
                                Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr, 
                                Liste_Command_FEETECH[Command_FEETECH_DONE].Reg_Addr);
                    #endif

                    *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_TIMEOUT;
                    FEETECH_Loop_State = 90;
                }
            } else if ((Timer_ms1 - Time_Of_Last_FEETECH_Received) > Com_FEETECH_Maxtime) {
                #ifdef FEETECH_PROTOCOL_DEBUG
                    printf("FEETECH Error: No response from ID %d (Reg 0x%02X)\r\n", 
                            Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr, 
                            Liste_Command_FEETECH[Command_FEETECH_DONE].Reg_Addr);
                #endif
                
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_TIMEOUT;
                FEETECH_Loop_State = 90;
            }
            break;

        case 30:
            val8 = 0;
            for (i = 2; i <= (FEETECH_Receive_Tab[3] + 2); i++)
                val8 += FEETECH_Receive_Tab[i];
            val8 = ~val8;
            if (val8 == FEETECH_Receive_Tab[FEETECH_Receive_Tab[3] + 3]) {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_OK;
                FEETECH_Loop_State = 31;
            } else {
                #ifdef FEETECH_PROTOCOL_DEBUG
                    // --- PRINT CHECKSUM ERROR ---
                    printf("FEETECH Error: Checksum mismatch from ID %d. Expected 0x%02X, got 0x%02X\r\n", 
                            Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr, 
                            val8, FEETECH_Receive_Tab[FEETECH_Receive_Tab[3] + 3]);
                #endif

                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_CHKSUM_ERROR;
                FEETECH_Loop_State = 90;
            }
            break;

        case 31:
            if ((Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_READ_DATA) &&
                (Liste_Command_FEETECH[Command_FEETECH_DONE].Data_Answer != NULL) ) {
                
                uint8_t *ptr_on_u8 = (uint8_t*)Liste_Command_FEETECH[Command_FEETECH_DONE].Data_Answer;
                uint8_t is_scs_word = (Liste_Command_FEETECH[Command_FEETECH_DONE].Protocol == FEETECH_PROTO_SCS && 
                                       Liste_Command_FEETECH[Command_FEETECH_DONE].Nb_Data == 2);

                // --- MODIFICATION ICI : GESTION ENDIANNESS LECTURE ---
                if (is_scs_word) {
                     // SCS (Big Endian) reçu : [0] = MSB, [1] = LSB.
                     // On veut le stocker en Little Endian en mémoire : [0] = LSB, [1] = MSB.
                     ptr_on_u8[0] = FEETECH_Receive_Tab[5 + 1]; // LSB (2ème octet reçu)
                     ptr_on_u8[1] = FEETECH_Receive_Tab[5 + 0]; // MSB (1er octet reçu)
                } 
                else {
                    // STS (Standard) : Copie directe
                    for (i = 0; i < Liste_Command_FEETECH[Command_FEETECH_DONE].Nb_Data; i++) {
                        ptr_on_u8[i] = FEETECH_Receive_Tab[i + 5];
                    }
                }
                // ----------------------------------------------------
            }
            FEETECH_Loop_State = 100;
            break;

        case 90:
            {
            u8 Garbage;
            while(Get_Uart1_Cmd(&Garbage));
            }
            FEETECH_Receive_Ptr = 0;
            FEETECH_Bytes_To_Ignore = 0;
            FEETECH_Cmd_Nb_Try ++;
            if (FEETECH_Cmd_Nb_Try < FEETECH_CMD_NB_MAX_TRY_SEND) {
                #ifdef FEETECH_PROTOCOL_DEBUG
                    // Print de la tentative de retry
                    printf("FEETECH: Retrying ID %d (Attempt %d/%d)\r\n", 
                            Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr, 
                            FEETECH_Cmd_Nb_Try + 1, FEETECH_CMD_NB_MAX_TRY_SEND);
                #endif

                // FEETECH_Loop_State = 10;
                // Com_FEETECH_Status = COM_FEETECH_IDDLE;
                FEETECH_IFG_Timer = Timer_ms1; // On lance le chrono
                FEETECH_Loop_State = 91;
            } else {
                #ifdef FEETECH_PROTOCOL_DEBUG
                    // Print de l'abandon final
                    printf("FEETECH Error: Command failed for ID %d after %d attempts.\r\n", 
                            Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr, 
                            FEETECH_CMD_NB_MAX_TRY_SEND);
                #endif

                FEETECH_Loop_State = 100;
            }
            break;
        
        case 91: 
            // Attente non-bloquante de 2ms avant de renvoyer la commande
            if ((Timer_ms1 - FEETECH_IFG_Timer) >= 3) {
                FEETECH_Loop_State = 10;
                Com_FEETECH_Status = COM_FEETECH_IDDLE;
            }
            break;

        case 100:
            Com_FEETECH_Status = COM_FEETECH_IDDLE;
            *((uint8_t*) Liste_Command_FEETECH[Command_FEETECH_DONE].Done) = 1;
            Command_FEETECH_DONE++;
            if (Command_FEETECH_DONE == FEETECH_CMD_LIST_SIZE)
                Command_FEETECH_DONE = 0;
            
            // --- AJOUT : Préparation du délai inter-commande ---
            // On réutilise Time_Of_Last_FEETECH_Received pour marquer le début du délai
            FEETECH_IFG_Timer = Timer_ms1; 
            FEETECH_Loop_State = 101; 
            break;

        // --- AJOUT : Nouvel état pour gérer le délai sans bloquer le CPU ---
        case 101:
            // On attend 2 millisecondes (vous pouvez tester 1, 2 ou 3 selon vos cartes)
            if ((Timer_ms1 - FEETECH_IFG_Timer) >= 3) {
                FEETECH_Loop_State = 0; // Le délai est passé, on peut traiter la commande suivante
            }
            break;

        default:
            FEETECH_Loop_State = 0;
            break;
    }
}

uint8_t RegisterLenFEETECH(uint8_t address) {
    switch (address) {
        case FEETECH_MODEL_L: case FEETECH_MODEL_H: case FEETECH_ID: case FEETECH_BAUD_RATE: case FEETECH_DELAY_TIME_RETURN: case FEETECH_LEVEL_RETURN: case FEETECH_MAX_TEMP_LIMIT: case FEETECH_MAX_INPUT_VOLT:
        case FEETECH_MIN_INPUT_VOLT: case FEETECH_SETTING_BYTE: case FEETECH_PROTECTION_ENABLE: case FEETECH_ALARM_LED: case FEETECH_CW_DEAD: case FEETECH_CCW_DEAD:
        case FEETECH_RESOLUTION: case FEETECH_MODE: case FEETECH_TORQUE_ENABLE: case FEETECH_LOCK: case FEETECH_PRESENT_VOLTAGE:
        case FEETECH_ACC: case FEETECH_PRESENT_TEMPERATURE: case FEETECH_MOVING: 
        case PUMP_CMD_1: case PUMP_CMD_2: case VALVE_CMD_1: case VALVE_CMD_2:
            return 1;
        case FEETECH_MIN_ANGLE_LIMIT_L:  case FEETECH_MAX_ANGLE_LIMIT_L: case FEETECH_MAX_TORQUE_LIMIT_L:
        case FEETECH_OFS_L: case FEETECH_MIN_START_TORQUE: case FEETECH_OVERLOAD_CURRENT_L: case FEETECH_GOAL_POSITION_L: case FEETECH_GOAL_TIME_L: case FEETECH_GOAL_SPEED_L:
        case FEETECH_TORQUE_LIMIT_L: case FEETECH_PRESENT_POSITION_L: case FEETECH_PRESENT_SPEED_L: case FEETECH_PRESENT_LOAD_L: case FEETECH_PRESENT_CURRENT_L: case ADDR_CURRENT_1_L: case ADDR_CURRENT_2_L:
            return 2;
        default:
            return 0;
    }
}

// Mise à jour de la fonction Add pour accepter le protocole
void Add_FEETECH_Cmd(uint8_t FEETECH_Addr, uint16_t Uart_Brg, uint8_t Command, uint8_t Reg_Addr, uint32_t Data_To_Send, void *Data_Answer, uint8_t Nb_Data, uint8_t *Status, void *Done, uint8_t Protocol) {
    Liste_Command_FEETECH[Command_FEETECH_TODO].Uart_Brg = Uart_Brg;
    Liste_Command_FEETECH[Command_FEETECH_TODO].FEETECH_Addr = FEETECH_Addr;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Command = Command;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Reg_Addr = Reg_Addr;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Data_To_Send = Data_To_Send;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Data_Answer = Data_Answer;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Nb_Data = Nb_Data;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Status = Status;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Done = Done;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Protocol = Protocol; // Stockage du type de protocole

    Command_FEETECH_TODO++;
    if (Command_FEETECH_TODO == FEETECH_CMD_LIST_SIZE)
        Command_FEETECH_TODO = 0;
}

// ================= WRAPPERS POUR STS (ANCIENS SERVOS) =================
void PutFEETECH(uint8_t id, uint8_t Reg, uint32_t Data) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy, FEETECH_PROTO_STS);
}

void PutFEETECH_Wait(uint8_t id, uint8_t Reg, uint32_t Data) {
    volatile uint8_t Done = 0;
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_STS);
    while (!Done) FEETECH_Loop();
}

void PutFEETECH_Ext_Done(uint8_t id, uint8_t Reg, uint32_t Data, void *Done) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_STS);
}

uint32_t GetFEETECH_Wait(uint8_t id, uint8_t Reg) {
    volatile uint8_t Done = 0;
    uint32_t Data_Answer = 0;
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, &Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_STS);
    while (!Done) FEETECH_Loop();
    return Data_Answer;
}

void GetFEETECH(uint8_t id, uint8_t Reg, void *Data_Answer) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy, FEETECH_PROTO_STS);
}

void GetFEETECH_Ext_Done(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_STS);
}

// ================= WRAPPERS POUR SCS (NOUVEAUX SERVOS) =================
// Utilisez ces fonctions pour le SCS0009
void PutFEETECH_SCS(uint8_t id, uint8_t Reg, uint32_t Data) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy, FEETECH_PROTO_SCS);
}

void PutFEETECH_Wait_SCS(uint8_t id, uint8_t Reg, uint32_t Data) {
    volatile uint8_t Done = 0;
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_SCS);
    while (!Done) FEETECH_Loop();
}

uint32_t GetFEETECH_Wait_SCS(uint8_t id, uint8_t Reg) {
    volatile uint8_t Done = 0;
    uint32_t Data_Answer = 0;
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, &Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_SCS);
    while (!Done) FEETECH_Loop();
    return Data_Answer;
}

void GetFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_SCS);
}

uint8_t FEETECH_All_Cmd_Done(void) {
    return (Command_FEETECH_TODO == Command_FEETECH_DONE);
}

void GetFEETECH_Ext_Done_With_Status(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done, uint8_t *Status) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), Status, Done, FEETECH_PROTO_STS);
}

void PutFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, uint32_t Data, void *Done) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_SCS);
}