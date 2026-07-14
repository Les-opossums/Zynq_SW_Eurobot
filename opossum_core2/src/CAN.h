#define CAN_DEVICE_ID	XPAR_XCANPS_0_DEVICE_ID

#define INTC		XScuGic

#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define CAN_INTR_VEC_ID		XPAR_XCANPS_1_INTR

#define BRPR_BAUD_PRESCALAR	9

#define BTR_SYNCJUMPWIDTH		0
#define BTR_SECOND_TIMESEGMENT	0
#define BTR_FIRST_TIMESEGMENT	7

#define CAN_MOTOR_FILTER_MASK 0x203

#define CAN_MOTOR_1_ID 0x201
#define CAN_MOTOR_2_ID 0x202
#define CAN_MOTOR_3_ID 0x203
#define CAN_MOTOR_4_ID 0x204

/* Maximum CAN frame length in word */
#define XCANPS_MAX_FRAME_SIZE_IN_WORDS (XCANPS_MAX_FRAME_SIZE / sizeof(u32))
#define FRAME_DATA_LENGTH	8 /* Frame Data field length */
#define ESC_TX_MESSAGE_ID 0x200

// #define DEBUG_CAN

extern XCanPs CanInstance; 

extern int motor1_current_order;
extern int motor2_current_order;
extern int motor3_current_order;
extern int motor4_current_order;

extern int angle_motor_1;
extern int angle_motor_2;
extern int angle_motor_3;
extern int angle_motor_4;
 
extern int torque_motor_1;
extern int torque_motor_2;
extern int torque_motor_3;
extern int torque_motor_4;
 
extern int speed_motor_1;
extern int speed_motor_2;
extern int speed_motor_3;
extern int speed_motor_4;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
    float motor4;
} ESC_Torque;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
    float motor4;
} ESC_Speed;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
    float motor4;
} ESC_Angle;

typedef struct {
    ESC_Torque torque;
    ESC_Speed speed;
    ESC_Angle angle;
} ESC_Info;

typedef struct {
    uint16_t id;             // the 11-bit message ID
    uint8_t buffer;          // the buffer the message is stored in
    uint8_t payload[8];      // the 8 bytes of data
    uint8_t valid_bytes;     // the number of valid bytes in the payload
} CAN_Message;

/**
 * @brief Statistiques d'erreurs et d'évènements bus, à des fins de monitoring/debug.
 *
 * Cumulatif depuis le dernier appel à CAN_ResetErrorStats() (ou depuis le
 * dernier Init_CAN()). Ne remplace pas un scope/analyseur CAN mais permet de
 * voir en un coup d'oeil si le bus est sain (via PlotJuggler/rqt_plot par
 * exemple, ou un simple print périodique).
 */
typedef struct {
    /* Compteurs d'erreurs élémentaires, un par bit du registre ESR */
    uint32_t ack_error_count;
    uint32_t bit_error_count;
    uint32_t stuff_error_count;
    uint32_t form_error_count;
    uint32_t crc_error_count;

    /* Compteurs d'évènements bus, issus du registre ISR */
    uint32_t bus_off_count;
    uint32_t rx_fifo_overflow_count;
    uint32_t rx_fifo_underflow_count;
    uint32_t arbitration_lost_count;

    /* Compteur agrégé, pratique pour un affichage rapide ou un seuil d'alerte */
    uint32_t total_error_count;

    /* Dernier masque brut reçu (utile en debug pour distinguer des erreurs
     * combinées survenues dans la même interruption) */
    uint32_t last_esr_value;

    /* Etat logiciel du driver (reflète CAN_IsEnabled()) */
    uint8_t bus_enabled;
} CAN_ErrorStats;

/**
 * @brief Statistiques d'erreurs bus courantes. Lecture seule depuis
 * l'extérieur du driver (à mettre à jour uniquement via les fonctions du
 * driver CAN).
 */
extern CAN_ErrorStats can_error_stats;

/**
 * @brief Initializes the CAN controller.
 * 
 * @return Returns XST_SUCCESS on success, or an error code on failure.
 */
int Init_CAN(void);

/**
 * @brief Configures the CAN acceptance filters.
 * 
 * @return Returns XST_SUCCESS on success, or an error code on failure.
 */
int CAN_configure_filters(void);

/**
 * @brief Creates a CAN message.
 * 
 * @param message Pointer to the CAN_Message structure to be filled.
 * @param id The 11-bit message ID.
 * @param buffer The buffer number for the message.
 * @param payload Pointer to the data payload of the message.
 * @param valid_bytes The number of valid bytes in the payload.
 */
void CAN_create_message(CAN_Message *message, uint16_t id, uint8_t buffer, uint8_t *payload, uint8_t valid_bytes);

/**
 * @brief Transmits a CAN frame.
 * 
 * @param InstancePtr Pointer to the XCanPs instance.
 */
void CAN_transmit(XCanPs *InstancePtr);

/**
 * This function configures the CAN controller with the following settings:
 * - Baud Rate Prescalar
 * - Bit Timing Register 0 (BTR0)
 * - Bit Timing Register 1 (BTR1)
 * 
 * @param	InstancePtr is a pointer to the XCanPs instance.
 * 
 * @return	None.
 * 
 * @note		None.
 */
int Config(XCanPs *InstancePtr);

/**
 * @brief Sends a CAN frame.
 * 
 * @param InstancePtr Pointer to the XCanPs instance.
 */
void SendFrame(XCanPs *InstancePtr);

/**
 * @brief Handler for the CAN send interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 */
void SendHandler(void *CallBackRef);

/**
 * @brief Handler for the CAN receive interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 */
void RecvHandler(void *CallBackRef);

/**
 * @brief Handler for the CAN error interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 * @param ErrorMask The error mask indicating the type of error.
 */
void ErrorHandler(void *CallBackRef, u32 ErrorMask);

/**
 * @brief Handler for the CAN event interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 * @param IntrMask The interrupt mask indicating the type of event.
 */
void EventHandler(void *CallBackRef, u32 IntrMask);

/**
 * @brief Transmits motor control commands via CAN.
 * 
 * @param motor1 Current order for motor 1.
 * @param motor2 Current order for motor 2.
 * @param motor3 Current order for motor 3.
 * @param motor4 Current order for motor 4.
 */
void CAN_transmit_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief Initializes the variables used for CAN motor control.
 */
void Init_CAN_MOTOR_variables(void);

/**
 * @brief Désactive le contrôleur CAN proprement.
 *
 * Coupe les interruptions du périphérique et bascule le coeur CAN en mode
 * Configuration : il n'émet plus, ne tente plus d'arbitrer le bus et ne
 * génère donc plus d'erreurs (ACK error, Bus-Off, ...). A utiliser lors d'un
 * arrêt d'urgence, quand les ESC ne sont plus alimentés et ne peuvent donc
 * plus ACKer les trames.
 *
 * Sans effet si le driver est déjà désactivé. Bloquant (attente courte du
 * changement de mode) : à appeler en contexte tâche, pas dans un handler
 * d'interruption imbriqué.
 */
void CAN_Disable(void);

/**
 * @brief Réactive le contrôleur CAN après un CAN_Disable().
 *
 * Réinitialise le coeur CAN (registres + compteurs d'erreurs matériels),
 * le reconfigure (baudrate, bit timing, filtres) puis repasse en mode
 * Normal et réactive les interruptions.
 *
 * Sans effet si le driver est déjà activé. Bloquant (attente courte du
 * changement de mode) : à appeler en contexte tâche, pas dans un handler
 * d'interruption imbriqué.
 */
void CAN_Enable(void);

/**
 * @brief Indique si le driver CAN est actuellement activé.
 *
 * @return TRUE si activé (mode Normal, interruptions actives), FALSE sinon
 * (ex : suite à un arrêt d'urgence).
 */
uint8_t CAN_IsEnabled(void);

/**
 * @brief Remet à zéro les statistiques d'erreurs bus (can_error_stats).
 *
 * Le champ bus_enabled est repositionné à l'état courant du driver, tous
 * les autres compteurs sont remis à 0.
 */
void CAN_ResetErrorStats(void);

/**
 * @brief Affiche les statistiques d'erreurs bus courantes via xil_printf.
 *
 * Utilitaire de debug rapide ; pour un monitoring continu (PlotJuggler,
 * rqt_plot, ...) préférer lire directement les champs de can_error_stats.
 */
void CAN_PrintErrorStats(void);