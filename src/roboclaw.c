#include "roboclaw.h"

void roboclaw_init(ROBOCLAW_t *driver) {
    // Dynamically assign uart_inst and baud rate if not provided (fallback defaults)
    if (driver->uart_inst == NULL) driver->uart_inst = uart1;
    if (driver->baud_rate == 0) driver->baud_rate = 38400; // Professional standard

    uart_init(driver->uart_inst, driver->baud_rate);
    uart_set_format(driver->uart_inst, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(driver->uart_inst, true);
    
    gpio_set_function(driver->tx_pin, UART_FUNCSEL_NUM(driver->uart_inst, driver->tx_pin));
    gpio_set_function(driver->rx_pin, UART_FUNCSEL_NUM(driver->uart_inst, driver->rx_pin));
}

bool roboclaw_is_ready_to_tx(ROBOCLAW_t *driver) {
    return uart_is_writable(driver->uart_inst);
}

// Internal standard serial command sender
static bool roboclaw_send_command(ROBOCLAW_t *driver, uint8_t command) {
    if (roboclaw_is_ready_to_tx(driver)) {
        // Removed wait_blocking! True non-blocking execution.
        uart_putc_raw(driver->uart_inst, command);
        return true;
    }
    return false;
}

bool roboclaw_motor_1_forward(ROBOCLAW_t *driver, uint8_t speed) {
    if (speed > ROBOCLAW_MAX_SPEED) speed = ROBOCLAW_MAX_SPEED;
    return roboclaw_send_command(driver, speed + 64);
}

bool roboclaw_motor_1_backward(ROBOCLAW_t *driver, uint8_t speed) {
    if (speed > ROBOCLAW_MAX_SPEED) speed = ROBOCLAW_MAX_SPEED;
    return roboclaw_send_command(driver, 64 - speed);
}

bool roboclaw_motor_2_forward(ROBOCLAW_t *driver, uint8_t speed) {
    if (speed > ROBOCLAW_MAX_SPEED) speed = ROBOCLAW_MAX_SPEED;
    return roboclaw_send_command(driver, 192 + speed);
}

bool roboclaw_motor_2_backward(ROBOCLAW_t *driver, uint8_t speed) {
    if (speed > ROBOCLAW_MAX_SPEED) speed = ROBOCLAW_MAX_SPEED;
    return roboclaw_send_command(driver, 192 - speed);
}

bool roboclaw_motor_1_stop(ROBOCLAW_t *driver) {
    return roboclaw_send_command(driver, 64);
}

bool roboclaw_motor_2_stop(ROBOCLAW_t *driver) {
    return roboclaw_send_command(driver, 192);
}

bool roboclaw_stop(ROBOCLAW_t *driver) {
    return roboclaw_send_command(driver, 0);
}