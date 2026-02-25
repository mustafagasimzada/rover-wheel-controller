#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

// Define the maximum speed for RoboClaw standard serial
#define ROBOCLAW_MAX_SPEED 63

typedef struct {
    uart_inst_t *uart_inst; // Make UART instance dynamic!
    uint8_t tx_pin;
    uint8_t rx_pin;
    uint32_t baud_rate;
} ROBOCLAW_t;

// Initialization
void roboclaw_init(ROBOCLAW_t *driver);

// Hardware status
bool roboclaw_is_ready_to_tx(ROBOCLAW_t *driver);

// Motor control functions
bool roboclaw_motor_1_forward(ROBOCLAW_t *driver, uint8_t speed);
bool roboclaw_motor_1_backward(ROBOCLAW_t *driver, uint8_t speed);
bool roboclaw_motor_2_forward(ROBOCLAW_t *driver, uint8_t speed);
bool roboclaw_motor_2_backward(ROBOCLAW_t *driver, uint8_t speed);

// Stopping functions
bool roboclaw_motor_1_stop(ROBOCLAW_t *driver);
bool roboclaw_motor_2_stop(ROBOCLAW_t *driver);
bool roboclaw_stop(ROBOCLAW_t *driver); // Stops both motors

#endif // ROBOCLAW_H