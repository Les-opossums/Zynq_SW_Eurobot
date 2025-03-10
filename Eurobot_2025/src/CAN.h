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

/* Maximum CAN frame length in word */
#define XCANPS_MAX_FRAME_SIZE_IN_WORDS (XCANPS_MAX_FRAME_SIZE / sizeof(u32))
#define FRAME_DATA_LENGTH	8 /* Frame Data field length */
#define ESC_TX_MESSAGE_ID 0x200

// #define DEBUG_CAN

extern XCanPs CanInstance; 

extern int motor1_current_order;
extern int motor2_current_order;
extern int motor3_current_order;

extern int angle_motor_1;
extern int angle_motor_2;
extern int angle_motor_3;
 
extern int torque_motor_1;
extern int torque_motor_2;
extern int torque_motor_3;
 
extern int speed_motor_1;
extern int speed_motor_2;
extern int speed_motor_3;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
} ESC_Torque;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
} ESC_Speed;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
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

int init_CAN(void);
int CAN_configure_filters(void);

void CAN_create_message(CAN_Message *message, uint16_t id, uint8_t buffer, uint8_t *payload, uint8_t valid_bytes);
void CAN_transmit(XCanPs *InstancePtr);

int Config(XCanPs *InstancePtr);
void SendFrame(XCanPs *InstancePtr);

void SendHandler(void *CallBackRef);
void RecvHandler(void *CallBackRef);

void ErrorHandler(void *CallBackRef, u32 ErrorMask);
void EventHandler(void *CallBackRef, u32 IntrMask);

void Can_Loop(void);

uint8_t Motor_cmd(void);
