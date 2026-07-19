#include "feetech_io.h"
#include "xstatus.h"
#include "xuartps_hw.h"
#include "sleep.h"
#include "../../TIMER_MANAGER/timer_manager.h"

#define COM_FEETECH_IDDLE           0x00
#define COM_FEETECH_SENDING         0x01
#define COM_FEETECH_WAIT_ANSWER     0x02

#define FEETECH_PROTO_STS           0 // Little Endian (STS3215...)
#define FEETECH_PROTO_SCS           1 // Big Endian (SCS0009...)

#define RX_TIMEOUT_MS_DEFAULT 20

/* Instance active : les wrappers bloquants historiques (PutFEETECH_Wait...)
 * ne prennent pas de contexte en parametre (bus FEETECH unique sur ce
 * robot), on garde donc un pointeur vers l'unique instance initialisee par
 * FEETECH_IO_Init(), exactement comme l'ancien code s'appuyait sur des
 * variables globales uniques. */
static feetech_io_context_t *ActiveCtx = NULL;

static uint8_t Com_FEETECH_Status = COM_FEETECH_IDDLE;
static uint32_t Time_Of_Last_FEETECH_Received = 0;
static uint32_t FEETECH_IFG_Timer = 0;
static uint32_t Com_FEETECH_Maxtime = 10;

static uint8_t FEETECH_Loop_State = 0;

static uint8_t FEETECH_Transmit_Tab[FEETECH_CMD_BUFF_LENGTH] = {0xFF, 0xFF, 0};
static uint8_t FEETECH_Transmit_Goal = 0;
static uint8_t FEETECH_Receive_Tab[FEETECH_CMD_BUFF_LENGTH];
static uint8_t FEETECH_Receive_Ptr = 0;

static uint8_t FEETECH_Bytes_To_Ignore = 0;

static FEETECH_Command Liste_Command_FEETECH[FEETECH_CMD_LIST_SIZE];
static uint8_t Command_FEETECH_TODO = 0;
static uint8_t Command_FEETECH_DONE = 0;
static uint8_t FEETECH_Cmd_Nb_Try = 0;
static uint8_t FEETECH_Dumy = 0;

/* forward */
static void FEETECH_Cmd_Send(feetech_io_context_t *ctx, FEETECH_Command *Cmd);

/* ═══════════════════════════════════════════════════════════════════════
 * Helpers bas niveau
 * ═══════════════════════════════════════════════════════════════════════ */

static void feetech_flush_rx(feetech_io_context_t *ctx) {
    uint8_t dummy;
    // 1. Vide le buffer logiciel (ring buffer du driver UART_PS)
    while (UART_PS_GetByte(ctx->uart, &dummy)) { }
    // 2. Vide sauvagement le FIFO materiel de l'UART (au cas ou un octet
    // serait coince dans le hardware, ex: echo residuel du bus half-duplex)
    while (XUartPs_IsReceiveData(ctx->uart->instance.Config.BaseAddress)) {
        XUartPs_ReadReg(ctx->uart->instance.Config.BaseAddress, XUARTPS_FIFO_OFFSET);
    }
}

/* "Emission terminee" = plus rien dans le ring buffer TX logiciel ET FIFO
 * materiel vide (registre a decalage sorti). Remplace l'ancien flag
 * feetech_tx_done, qui etait pose par l'event handler XUARTPS_EVENT_SENT_DATA
 * (plus disponible ici : driver_uart_ps.c ne relaie pas cet evenement). */
static u8 feetech_tx_complete(feetech_io_context_t *ctx) {
    return (ctx->uart->tx_buf.i_done == ctx->uart->tx_buf.i_todo) &&
           XUartPs_IsTransmitEmpty(&ctx->uart->instance);
}

/* ═══════════════════════════════════════════════════════════════════════
 * 1. Initialisation (cf IO_Manager)
 * ═══════════════════════════════════════════════════════════════════════ */

int FEETECH_IO_Init(void *instance) {
    feetech_io_context_t *ctx = (feetech_io_context_t *)instance;
    int Status;

    Status = XGpio_Initialize(&ctx->dir_gpio, ctx->dir_gpio_device_id);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XGpio_SetDataDirection(&ctx->dir_gpio, ctx->dir_gpio_channel, 0x0); // en sortie
    XGpio_DiscreteWrite(&ctx->dir_gpio, ctx->dir_gpio_channel, FEETECH_DIR_RX);

    Com_FEETECH_Status = COM_FEETECH_IDDLE;
    Time_Of_Last_FEETECH_Received = 0;
    FEETECH_IFG_Timer = 0;
    Com_FEETECH_Maxtime = 10;
    FEETECH_Loop_State = 0;
    FEETECH_Transmit_Goal = 0;
    FEETECH_Receive_Ptr = 0;
    FEETECH_Bytes_To_Ignore = 0;
    Command_FEETECH_TODO = 0;
    Command_FEETECH_DONE = 0;
    FEETECH_Cmd_Nb_Try = 0;

    ActiveCtx = ctx;

    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════
 * 2. Envoi d'une commande (protocole FEETECH, checksum + endianness)
 * ═══════════════════════════════════════════════════════════════════════ */

static void FEETECH_Cmd_Send(feetech_io_context_t *ctx, FEETECH_Command *Cmd) {
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

        // --- GESTION ENDIANNESS ---
        if (Cmd->Protocol == FEETECH_PROTO_SCS && Cmd->Nb_Data == 2) {
            // SCS (Big Endian) : on envoie d'abord le poids FORT (MSB)
            FEETECH_Transmit_Tab [6] = (uint8_t)((Data_To_Send >> 8) & 0xFF);
            checksum_sum += FEETECH_Transmit_Tab [6];
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

    feetech_flush_rx(ctx);

    FEETECH_Receive_Ptr = 0;
    FEETECH_Bytes_To_Ignore = FEETECH_Transmit_Goal;

    XGpio_DiscreteWrite(&ctx->dir_gpio, ctx->dir_gpio_channel, FEETECH_DIR_TX);

    UART_PS_SendBuffer(ctx->uart, FEETECH_Transmit_Tab, FEETECH_Transmit_Goal);

    Com_FEETECH_Status = COM_FEETECH_SENDING;
    Time_Of_Last_FEETECH_Received = Timer_ms1;
    Com_FEETECH_Maxtime = RX_TIMEOUT_MS_DEFAULT;
}

/* ═══════════════════════════════════════════════════════════════════════
 * 3. Boucle protocole (cf IO_Manager_Update) — machine a etats
 * ═══════════════════════════════════════════════════════════════════════ */

void FEETECH_IO_Update(void *instance) {
    feetech_io_context_t *ctx = (feetech_io_context_t *)instance;
    uint8_t val8, i;
    uint8_t b;

    // Recuperation des octets UART
    while (UART_PS_GetByte(ctx->uart, &b)) {
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

    switch(FEETECH_Loop_State) {
        case 0:
            if (Command_FEETECH_TODO != Command_FEETECH_DONE){
                FEETECH_Loop_State++;
            }
            break;

        case 1:
            FEETECH_Cmd_Nb_Try = 0;
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
            FEETECH_Cmd_Send(ctx, &Liste_Command_FEETECH[Command_FEETECH_DONE]);
            FEETECH_Loop_State++;
            break;

        case 11:
            if (feetech_tx_complete(ctx)) {
                usleep(150);
                XGpio_DiscreteWrite(&ctx->dir_gpio, ctx->dir_gpio_channel, FEETECH_DIR_RX);
                FEETECH_Receive_Ptr = 0;
                Time_Of_Last_FEETECH_Received = Timer_ms1;
                FEETECH_Loop_State = 20;
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
                    FEETECH_LOG("FEETECH Error: Timeout (Partial Rx) on ID %d (Reg 0x%02X)\r\n",
                            Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr,
                            Liste_Command_FEETECH[Command_FEETECH_DONE].Reg_Addr);

                    *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_TIMEOUT;
                    FEETECH_Loop_State = 90;
                }
            } else if ((Timer_ms1 - Time_Of_Last_FEETECH_Received) > Com_FEETECH_Maxtime) {
                FEETECH_LOG("FEETECH Error: No response from ID %d (Reg 0x%02X)\r\n",
                        Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr,
                        Liste_Command_FEETECH[Command_FEETECH_DONE].Reg_Addr);

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
                FEETECH_LOG("FEETECH Error: Checksum mismatch from ID %d. Expected 0x%02X, got 0x%02X\r\n",
                        Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr,
                        val8, FEETECH_Receive_Tab[FEETECH_Receive_Tab[3] + 3]);

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

                if (is_scs_word) {
                     // SCS (Big Endian) recu : [0] = MSB, [1] = LSB.
                     // Stocke en Little Endian en memoire : [0] = LSB, [1] = MSB.
                     ptr_on_u8[0] = FEETECH_Receive_Tab[5 + 1];
                     ptr_on_u8[1] = FEETECH_Receive_Tab[5 + 0];
                }
                else {
                    for (i = 0; i < Liste_Command_FEETECH[Command_FEETECH_DONE].Nb_Data; i++) {
                        ptr_on_u8[i] = FEETECH_Receive_Tab[i + 5];
                    }
                }
            }
            FEETECH_Loop_State = 100;
            break;

        case 90:
            {
                uint8_t Garbage;
                while(UART_PS_GetByte(ctx->uart, &Garbage));
            }
            FEETECH_Receive_Ptr = 0;
            FEETECH_Bytes_To_Ignore = 0;
            FEETECH_Cmd_Nb_Try ++;
            if (FEETECH_Cmd_Nb_Try < FEETECH_CMD_NB_MAX_TRY_SEND) {
                FEETECH_LOG("FEETECH: Retrying ID %d (Attempt %d/%d)\r\n",
                        Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr,
                        FEETECH_Cmd_Nb_Try + 1, FEETECH_CMD_NB_MAX_TRY_SEND);

                FEETECH_IFG_Timer = Timer_ms1;
                FEETECH_Loop_State = 91;
            } else {
                FEETECH_LOG("FEETECH Error: Command failed for ID %d after %d attempts.\r\n",
                        Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr,
                        FEETECH_CMD_NB_MAX_TRY_SEND);

                FEETECH_Loop_State = 100;
            }
            break;

        case 91:
            // Attente non-bloquante de quelques ms avant de renvoyer la commande
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

            FEETECH_IFG_Timer = Timer_ms1;
            FEETECH_Loop_State = 101;
            break;

        case 101:
            // Delai inter-commande, sans bloquer le CPU (1 a 3 ms selon les cartes)
            if ((Timer_ms1 - FEETECH_IFG_Timer) >= 3) {
                FEETECH_Loop_State = 0;
            }
            break;

        default:
            FEETECH_Loop_State = 0;
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * 4. Table des longueurs de registre
 * ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════
 * 5. File de commandes
 * ═══════════════════════════════════════════════════════════════════════ */

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
    Liste_Command_FEETECH[Command_FEETECH_TODO].Protocol = Protocol;

    Command_FEETECH_TODO++;
    if (Command_FEETECH_TODO == FEETECH_CMD_LIST_SIZE)
        Command_FEETECH_TODO = 0;
}

// ================= WRAPPERS POUR STS (protocole "petit-endian") =================
void PutFEETECH(uint8_t id, uint8_t Reg, uint32_t Data) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy, FEETECH_PROTO_STS);
}

void PutFEETECH_Wait(uint8_t id, uint8_t Reg, uint32_t Data) {
    volatile uint8_t Done = 0;
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_STS);
    while (!Done) FEETECH_IO_Update(ActiveCtx);
}

void PutFEETECH_Ext_Done(uint8_t id, uint8_t Reg, uint32_t Data, void *Done) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_STS);
}

uint32_t GetFEETECH_Wait(uint8_t id, uint8_t Reg) {
    volatile uint8_t Done = 0;
    uint32_t Data_Answer = 0;
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_READ_DATA, Reg, 0, &Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_STS);
    while (!Done) FEETECH_IO_Update(ActiveCtx);
    return Data_Answer;
}

void GetFEETECH(uint8_t id, uint8_t Reg, void *Data_Answer) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy, FEETECH_PROTO_STS);
}

void GetFEETECH_Ext_Done(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_STS);
}

void GetFEETECH_Ext_Done_With_Status(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done, uint8_t *Status) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), Status, Done, FEETECH_PROTO_STS);
}

// ================= WRAPPERS POUR SCS (protocole "grand-endian" 2 octets) =================
void PutFEETECH_SCS(uint8_t id, uint8_t Reg, uint32_t Data) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy, FEETECH_PROTO_SCS);
}

void PutFEETECH_Wait_SCS(uint8_t id, uint8_t Reg, uint32_t Data) {
    volatile uint8_t Done = 0;
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_SCS);
    while (!Done) FEETECH_IO_Update(ActiveCtx);
}

void PutFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, uint32_t Data, void *Done) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_SCS);
}

uint32_t GetFEETECH_Wait_SCS(uint8_t id, uint8_t Reg) {
    volatile uint8_t Done = 0;
    uint32_t Data_Answer = 0;
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_READ_DATA, Reg, 0, &Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done), FEETECH_PROTO_SCS);
    while (!Done) FEETECH_IO_Update(ActiveCtx);
    return Data_Answer;
}

void GetFEETECH_Ext_Done_SCS(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done) {
    Add_FEETECH_Cmd(id, 0, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done, FEETECH_PROTO_SCS);
}

uint8_t FEETECH_All_Cmd_Done(void) {
    return (Command_FEETECH_TODO == Command_FEETECH_DONE);
}
