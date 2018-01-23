/* IFE 2018
 * Dashboard Software
 * Andrew Smith
 */

/* INCLUDES */
#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

/* DEFINITIONS */
#define TRUE         0x01
#define FALSE        0x00

#define LED_ON       LOW
#define LED_OFF      HIGH

#define CL_LEVEL_1   0x01
#define CL_LEVEL_2   0x02
#define CL_LEVEL_3   0x03
#define CL_LEVEL_4   0x04
#define CL_LEVEL_5   0x05
#define CL_LEVEL_6   0x06

#define UART_TIMEOUT 100
#define UART_BUF_LEN 4
#define UART_MSG_ID  0
#define UART_TC_STAT 1
#define UART_CL_STAT 2
#define UART_CRC     3
#define UART_BTC     1
#define WHEEL_STAT_MSG 0x55
#define WHEEL_ERR_MSG  0x66
#define DASH_STAT_MSG  0x00
#define DASH_ERR_MSG   0x11

#define UART_WHEEL_ERR     1
#define UART_INVALID_DATA  2
#define UART_TIMEOUT_ERR   3

/* PINS */
#define BUZZER_ENABLE               10
#define MOSI                        11
#define MISO                        12
#define SCK                         13
#define WHEEL_LED                   14
#define AIL_LED                     15
#define BRAKE_THROTTLE_ERR_LED      16
#define ENABLE                      17
#define IMD_LED                     18
#define RXD                         2
#define TXD                         1
#define CS_CAN                      4
#define INT_CAN                     2
#define LIVE                        7


/* GLOBAL VARIABLES */
volatile uint8_t wheel_connected = FALSE;
volatile uint8_t precharge_complete = FALSE;
volatile uint8_t buzzer_on = FALSE;
volatile uint8_t ready_to_drive_complete = FALSE;
volatile uint8_t wheel_current_level = 0x00;
volatile uint8_t wheel_tc_en = FALSE;
volatile uint8_t wheel_brake_throttle = 0;
volatile uint8_t uart_rx_buffer[UART_BUF_LEN] = {0x00,0x00,0x00,0x00};
volatile uint8_t uart_tx_buffer[UART_BUF_LEN] = {0x00,0x00,0x00,0x00};


/* FUNCTION DECLARATIONS */
uint8_t set_led(uint8_t led, uint8_t state);
uint8_t can_send_msg(void);
void    can_recieve_msg(void);
uint8_t uart_send_brake_throttle_conflict(void);
uint8_t uart_recieve_wheel_info(void);
uint8_t crc(volatile uint8_t* buff, uint8_t len);


void setup() {
    noInterrupts();
    /* ----- PIN INIT ----- */
    /* LEDS - Output */
    pinMode(WHEEL_LED, OUTPUT);
    pinMode(AIL_LED, OUTPUT);
    pinMode(BRAKE_THROTTLE_ERR_LED, OUTPUT);
    pinMode(IMD_LED, OUTPUT);
    pinMode(LIVE, OUTPUT);

    /* Buzzer - Output */
    pinMode(BUZZER_ENABLE, OUTPUT);
    digitalWrite(BUZZER_ENABLE, LOW);

    /* Enable - Input from Brake Board */
    pinMode(ENABLE, INPUT);

    /* Interrupt Pins */
    pinMode(INT_CAN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_CAN), can_recieve_msg, FALLING);


    /* ----- UART INIT ----- */
    Serial.begin(38400, SERIAL_8N1);
    Serial.setTimeout(UART_TIMEOUT);

    
    /* ----- CAN INIT ----- */
    /* TODO */


    /* ----- Enable Interrupts ----- */
    interrupts();
}


void loop() {
    int buzzer_counter = 0;

    /* ----- Ready to Drive Sound (RTDS) ----- */
    if(precharge_complete == TRUE && ready_to_drive_complete == FALSE) {
        if(buzzer_on == FALSE) digitalWrite(BUZZER_ENABLE, HIGH);
        if(buzzer_counter++ > 3000) {
            digitalWrite(BUZZER_ENABLE, LOW);
            ready_to_drive_complete = FALSE;
        }
    }
}



/* FUNCTION IMPLEMENTATIONS */
uint8_t set_led(uint8_t led, uint8_t state) {
    switch(led){
        case WHEEL_LED: digitalWrite(WHEEL_LED, state);
        case AIL_LED: digitalWrite(AIL_LED, state);
        case BRAKE_THROTTLE_ERR_LED: digitalWrite(BRAKE_THROTTLE_ERR_LED, state);
        case IMD_LED: digitalWrite(IMD_LED, state);
        case LIVE: digitalWrite(LIVE, state);
        default:;
    }
    return 0;
}

uint8_t can_send_msg(void) {
    /* TODO */
    return 0;
}

void    can_recieve_msg(void) {
    set_led(LIVE, LED_ON);
    /* TODO */
    set_led(LIVE, LED_OFF);
}

uint8_t uart_send_brake_throttle_conflict(void) {
    uart_tx_buffer[UART_MSG_ID] = DASH_STAT_MSG;
    uart_tx_buffer[UART_BTC] = wheel_brake_throttle;
    uart_tx_buffer[UART_CRC] = crc(uart_tx_buffer, UART_BUF_LEN - 1);
    return Serial.write((char*)uart_tx_buffer, UART_BUF_LEN);
}

uint8_t uart_recieve_wheel_info(void) {
    if(Serial.readBytes((char*)uart_rx_buffer, UART_BUF_LEN) == UART_BUF_LEN) {
        if(uart_rx_buffer[UART_CRC] == crc(uart_rx_buffer, UART_BUF_LEN - 1)) {
            if(uart_rx_buffer[UART_MSG_ID] == WHEEL_STAT_MSG) {
                /* Message is Valid, Update Variables */
                wheel_current_level = uart_rx_buffer[UART_CL_STAT];
                wheel_tc_en = uart_rx_buffer[UART_TC_STAT];
                wheel_connected = TRUE;
                return 0;
            }
            else if(uart_rx_buffer[UART_MSG_ID] == WHEEL_ERR_MSG) {
                wheel_connected = TRUE;
                return UART_WHEEL_ERR;
            }
        }
        else {
            wheel_connected = TRUE;
            return UART_INVALID_DATA;
        }
    }
    wheel_connected = FALSE;
    return UART_TIMEOUT_ERR;
}

uint8_t crc(uint8_t* buff, uint8_t len) {
    uint8_t check_sum = 0xFF;
    for(uint8_t i = 0; i < len; i++) {
        check_sum ^= buff[i];
        check_sum = check_sum << 1;
    }
    return check_sum;
}

