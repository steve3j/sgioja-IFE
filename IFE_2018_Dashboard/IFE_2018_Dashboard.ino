/* IFE 2018
 * Dashboard Software
 * Andrew Smith
 */

/* INCLUDES */


/* DEFINITIONS */
#define TRUE        0x01
#define FALSE       0x00

#define LED_ON      0x00
#define LED_OFF     0x01

#define UART_ACK    0x01
#define UART_NACK   0x00

#define C_LEVEL_1   0x01
#define C_LEVEL_2   0x02
#define C_LEVEL_3   0x03
#define C_LEVEL_4   0x04
#define C_LEVEL_5   0x05
#define C_LEVEL_6   0x06

/* PINS */
#define BUZZER_ENABLE               10
#define MOSI                        11
#define MISO                        12
#define SCK                         13
#define WHEEL_CATHODE               14
#define AIL_CATHODE                 15
#define BRAKE_THROTTLE_ERR_CATHODE  16
#define ENABLE                      17
#define IMD_CATHODE                 18
#define RXD                         2
#define TXD                         1
#define CS_CAN                      4
#define INT_CAN                     2
#define LIVE                        7


/* GLOBAL VARIABLES */
volatile uint8_t wheel_connected = FALSE;
volatile uint8_t precharge_complete = FALSE;
volatile uint8_t wheel_current_level = 0x00;
volatile uint8_t wheel_tc_en = FALSE;
volatile uint8_t wheel_brake_throttle;
volatile uint8_t uart_rx_buffer[5] = {0x00,0x00,0x00,0x00};
volatile uint8_t uart_tx_buffer[5] = {0x00,0x00,0x00,0x00};


/* FUNCTION DECLARATIONS */
uint8_t set_led(uint8_t led, uint8_t state);
uint8_t can_send_msg(void);
uint8_t can_recieve_msg(void);
uint8_t uart_send_ack_to_wheel(void);
uint8_t uart_send_brake_throttle_conflict(void);
uint8_t crc(uint8_t* buff, uint8_t len);


/* */

void setup() {
  /* PIN INIT */

  /* CAN INIT */

  /* UART INIT */

}

void loop() {
  

}

/* Function Implementations */
