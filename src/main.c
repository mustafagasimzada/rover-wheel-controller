#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "port_common.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"
#include "hardware/i2c.h"
#include "dwm_pico_AS5600.h"
#include "roboclaw.h"

#define WHEEL_NUMBER 1

/* Per wheel ip configuration */
#define BASE_IP_LAST_OCTET  30
#define BASE_MAC_LAST_BYTE  0x30
#define BASE_PORT           8000

#define WHEEL_IP_LAST_OCTET (BASE_IP_LAST_OCTET + ((WHEEL_NUMBER - 1) * 2))
#define WHEEL_MAC_LAST_BYTE (BASE_MAC_LAST_BYTE  + (WHEEL_NUMBER - 1))
#define WHEEL_PORT          (BASE_PORT            + (WHEEL_NUMBER - 1))
#define WHEEL_SOCKET_NUMBER ((WHEEL_NUMBER - 1) % 4)

/* Clock and Buffers */
#define PLL_SYS_KHZ          (133 * 1000)
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)
#define LED_PIN               25
#define PWM_WRAP              64

/* JSON constraints */
#define MAX_JSON_SIZE  512
#define MAX_KEY_SIZE   32
#define MAX_VALUE_SIZE 64
#define MAX_JSON_PAIRS 10

/* I2C configuration */
#define I2C1_SDA_PIN 10
#define I2C1_SCL_PIN 11

/* Connection settings */
#define CONNECTION_TIMEOUT          2000
#define MAX_SOCKETS                 4
#define CONNECTION_MONITOR_INTERVAL 500

/* Steering & Drive Parameters */
#define STEERING_DEADBAND  2
#define STEERING_KP        0.8f
#define STEERING_KI        0.001f
#define STEERING_KD        0.005f
#define MAX_INTEGRAL       1000.0f
#define MAX_POSITION       360
#define MAX_DRIVE_PWM      (PWM_WRAP)
#define DRIVE_RAMP_RATE    100

#define ENABLE_DEBUG 0
#if ENABLE_DEBUG
  #define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
  #define DEBUG_PRINT(fmt, ...) 
#endif

// Calibration offsets
const int encoder_offsets[4] = {136, 129, 157, 150};
#define ENCODER_OFFSET encoder_offsets[WHEEL_NUMBER - 1]

// Global state and control variables
volatile float    integral_error        = 0.0f;
volatile float    last_error            = 0.0f;
volatile uint32_t last_update_time      = 0;
volatile uint16_t target_drive_speed    = 0;
volatile uint16_t current_drive_speed   = 0;
volatile bool     drive_forward         = true;
volatile int32_t  target_steering_angle = 0;
volatile bool     steering_active       = false;
volatile int32_t  current_steering_speed = 0;
volatile int32_t  current_steering_angle = 0;

bool     connection_active    = false;
uint32_t last_connection_time = 0;

typedef struct {
    uint8_t  sock_num;
    uint8_t  state;
    uint32_t last_activity;
    bool     is_server;
} socket_info_t;

typedef struct {
    char key[MAX_KEY_SIZE];
    char value[MAX_VALUE_SIZE];
} JsonPair;

socket_info_t socket_pool[MAX_SOCKETS] = {0};

as5600_t steering_encoder = {
    .i2c_inst = i2c1,
    .sda      = I2C1_SDA_PIN,
    .clk      = I2C1_SCL_PIN
};

ROBOCLAW_t roboclaw_motor = {
    .rx_pin = 9,
    .tx_pin = 8
};

wiz_NetInfo net_info = {
    .mac  = {0x00, 0x08, 0xdc, 0x16, 0xed, WHEEL_MAC_LAST_BYTE},
    .ip   = {10, 42, 0, WHEEL_IP_LAST_OCTET},
    .sn   = {255, 255, 255, 0},
    .gw   = {10, 42, 0, 254},
    .dns  = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

static uint8_t  ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {0};
static uint16_t local_port = WHEEL_PORT;

// Function Prototypes
void    init_socket_pool(void);
void    close_socket_gracefully(uint8_t sock_idx);
void    monitor_sockets(void);
void    setup_encoder(void);
void    setup_motors(void);
static void set_clock_khz(void);
int32_t process_tcp_server(uint8_t *buf, uint16_t port);
void    process_json_message(char *json_str, char *response_str);
int     parse_json(char *json_str, JsonPair *pairs, int max_pairs);
void    create_json_response(char *json_str, int max_size, JsonPair *pairs, int pair_count);
void    halt_motors(void);
void    update_steering_control(void);
void    update_drive_control(void);
int     calculateError(int target_position, int current_position, int max_position);


int main() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    setup_motors();
    setup_encoder();
    set_clock_khz();
    stdio_init_all();

    printf("Rover Wheel Controller initializing...\n");

    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    network_initialize(net_info);
    init_socket_pool();

    printf("Server bound to %d.%d.%d.%d:%d\n",
           net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3], local_port);

    // FIX: Initialize last_update_time so first PID dt is reasonable
    last_update_time      = to_ms_since_boot(get_absolute_time());
    last_connection_time  = to_ms_since_boot(get_absolute_time());

    while (true) {
        process_tcp_server(ethernet_buf, local_port);
        monitor_sockets();

        if (steering_active) update_steering_control();
        update_drive_control();

        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (connection_active && (current_time - last_connection_time > CONNECTION_TIMEOUT)) {
            DEBUG_PRINT("Watchdog: Connection timeout. Halting motors.\n");
            connection_active = false;
            halt_motors();
        }

        sleep_ms(5);
    }
}


int calculateError(int target_position, int current_position, int max_position) {
    int error = target_position - current_position;
    if (abs(error) > max_position / 2) {
        if (error > 0) error -= max_position;
        else           error += max_position;
    }
    return error;
}

void setup_motors(void) {
    roboclaw_init(&roboclaw_motor);
}

void setup_encoder(void) {
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
}

void halt_motors(void) {
    // FIX: Pass pointer to roboclaw_motor on all roboclaw calls
    roboclaw_stop(&roboclaw_motor);
    current_drive_speed  = 0;
    target_drive_speed   = 0;
    steering_active      = false;
    current_steering_speed = 0;
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
}

void update_drive_control(void) {
    static uint32_t last_command_time = 0;
    uint32_t current_time = time_us_32();

    bool should_resend = (current_time - last_command_time) >= 20000;

    if (!should_resend && current_drive_speed == target_drive_speed) return;

    if (current_drive_speed < target_drive_speed) {
        current_drive_speed = (target_drive_speed - current_drive_speed > DRIVE_RAMP_RATE)
                              ? current_drive_speed + DRIVE_RAMP_RATE
                              : target_drive_speed;
    } else if (current_drive_speed > target_drive_speed) {
        current_drive_speed = (current_drive_speed - target_drive_speed > DRIVE_RAMP_RATE)
                              ? current_drive_speed - DRIVE_RAMP_RATE
                              : target_drive_speed;
    }

    if (should_resend || current_drive_speed != target_drive_speed) {
        // FIX: Pass pointer to roboclaw_motor
        if (roboclaw_is_ready_to_tx(&roboclaw_motor)) {
            if (current_drive_speed == 0) {
                roboclaw_motor_1_stop(&roboclaw_motor);
            } else if (drive_forward) {
                roboclaw_motor_1_forward(&roboclaw_motor, current_drive_speed);
            } else {
                roboclaw_motor_1_backward(&roboclaw_motor, current_drive_speed);
            }
            last_command_time = current_time;
        }
    }
}

void update_steering_control(void) {
    static uint32_t last_command_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if ((current_time - last_command_time) < 20) return;

    current_steering_angle = as5600_angl_to_degr(
        as5600_read_raw_angl(&steering_encoder), 0, 4095) + ENCODER_OFFSET;

    int32_t error = calculateError(target_steering_angle, current_steering_angle, MAX_POSITION);

    if (abs(error) <= STEERING_DEADBAND) {
        // FIX: Pass pointer to roboclaw_motor
        if (roboclaw_is_ready_to_tx(&roboclaw_motor)) {
            roboclaw_motor_2_stop(&roboclaw_motor);
            current_steering_speed = 0;
            last_command_time = current_time;
        }
        integral_error = 0;
        return;
    }

    float dt = (current_time - last_update_time) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.02f;  // FIX: also clamp unreasonably large dt

    float p_term = STEERING_KP * error;
    integral_error += error * dt;

    // Anti-windup
    if (integral_error >  MAX_INTEGRAL) integral_error =  MAX_INTEGRAL;
    if (integral_error < -MAX_INTEGRAL) integral_error = -MAX_INTEGRAL;

    float i_term = STEERING_KI * integral_error;
    float d_term = STEERING_KD * ((error - last_error) / dt);
    float pid_output = p_term + i_term + d_term;

    uint8_t MIN_STEERING_SPEED = 10;
    uint8_t MAX_STEERING_SPEED = 60;
    uint8_t steering_speed = (uint8_t)fmin(fmax(fabs(pid_output),
                                                 MIN_STEERING_SPEED),
                                           MAX_STEERING_SPEED);

    // FIX: Pass pointer to roboclaw_motor
    if (roboclaw_is_ready_to_tx(&roboclaw_motor)) {
        if (pid_output > 0) {
            roboclaw_motor_2_forward(&roboclaw_motor, steering_speed);
            current_steering_speed =  steering_speed;
        } else {
            roboclaw_motor_2_backward(&roboclaw_motor, steering_speed);
            current_steering_speed = -steering_speed;
        }
        last_command_time = current_time;
    }

    last_error       = error;
    last_update_time = current_time;
}


void process_json_message(char *json_str, char *response_str) {
    JsonPair received_pairs[MAX_JSON_PAIRS];
    JsonPair response_pairs[MAX_JSON_PAIRS];
    int pair_count;

    connection_active    = true;
    last_connection_time = to_ms_since_boot(get_absolute_time());

    DEBUG_PRINT("Received JSON: %s\n", json_str);

    pair_count = parse_json(json_str, received_pairs, MAX_JSON_PAIRS);
    if (pair_count <= 0) {
        DEBUG_PRINT("Invalid JSON received\n");
        strcpy(response_str, "{\"error\":\"invalid_json\"}");
        return;
    }

    int  response_count = 0;
    bool led_command_received      = false;
    bool steering_command_received = false;
    bool drive_command_received    = false;

    for (int i = 0; i < pair_count; i++) {
        DEBUG_PRINT("Key: %s, Value: %s\n", received_pairs[i].key, received_pairs[i].value);

        if (strcmp(received_pairs[i].key, "topic") == 0 &&
            strcmp(received_pairs[i].value, "led_state") == 0) {

            led_command_received = true;
            for (int j = 0; j < pair_count; j++) {
                if (strcmp(received_pairs[j].key, "state") == 0) {
                    if (strcmp(received_pairs[j].value, "on")   == 0 ||
                        strcmp(received_pairs[j].value, "1")    == 0 ||
                        strcmp(received_pairs[j].value, "true") == 0) {
                        gpio_put(LED_PIN, 1);
                        DEBUG_PRINT("LED turned ON\n");
                        strcpy(response_pairs[response_count].key,   "led_state");
                        strcpy(response_pairs[response_count].value, "on");
                        response_count++;
                    } else if (strcmp(received_pairs[j].value, "off")   == 0 ||
                               strcmp(received_pairs[j].value, "0")     == 0 ||
                               strcmp(received_pairs[j].value, "false") == 0) {
                        gpio_put(LED_PIN, 0);
                        DEBUG_PRINT("LED turned OFF\n");
                        strcpy(response_pairs[response_count].key,   "led_state");
                        strcpy(response_pairs[response_count].value, "off");
                        response_count++;
                    }
                    break;
                }
            }
        }

        if (strcmp(received_pairs[i].key, "topic") == 0 &&
            strcmp(received_pairs[i].value, "drive_speed") == 0) {

            drive_command_received = true;
            for (int j = 0; j < pair_count; j++) {
                if (strcmp(received_pairs[j].key, "speed") == 0) {
                    long speed_val = strtol(received_pairs[j].value, NULL, 10);
                    if (speed_val == 0) {
                        target_drive_speed = 0;
                    } else if (speed_val < 0) {
                        target_drive_speed = (uint16_t)((-speed_val * MAX_DRIVE_PWM) / 100);
                        drive_forward      = false;
                        gpio_put(LED_PIN, 1);
                    } else {
                        target_drive_speed = (uint16_t)((speed_val * MAX_DRIVE_PWM) / 100);
                        drive_forward      = true;
                        gpio_put(LED_PIN, 0);
                    }
                    strcpy(response_pairs[response_count].key, "drive_speed");
                    sprintf(response_pairs[response_count].value, "%d",
                            drive_forward ? current_drive_speed : -(int)current_drive_speed);
                    response_count++;
                    break;
                }
            }
        }

        if (strcmp(received_pairs[i].key, "topic") == 0 &&
            strcmp(received_pairs[i].value, "steering_angle") == 0) {

            steering_command_received = true;
            for (int j = 0; j < pair_count; j++) {
                if (strcmp(received_pairs[j].key, "angle") == 0) {
                    long angle_val          = strtol(received_pairs[j].value, NULL, 10);
                    target_steering_angle   = angle_val;
                    steering_active         = true;
                    DEBUG_PRINT("New steering target angle: %d\n", target_steering_angle);

                    strcpy(response_pairs[response_count].key, "target_steering_angle");
                    sprintf(response_pairs[response_count].value, "%d", target_steering_angle);
                    response_count++;

                    strcpy(response_pairs[response_count].key, "current_steering_angle");
                    sprintf(response_pairs[response_count].value, "%d",
                            as5600_angl_to_degr(as5600_read_raw_angl(&steering_encoder), 0, 4095)
                            + ENCODER_OFFSET);
                    response_count++;
                    break;
                }
            }
        }

        if (strcmp(received_pairs[i].key, "command") == 0 &&
            (strcmp(received_pairs[i].value, "get_status")  == 0 ||
             strcmp(received_pairs[i].value, "heartbeat")   == 0)) {

            strcpy(response_pairs[response_count].key,   "status");
            strcpy(response_pairs[response_count].value, "running");
            response_count++;

            strcpy(response_pairs[response_count].key, "led_state");
            strcpy(response_pairs[response_count].value, gpio_get(LED_PIN) ? "on" : "off");
            response_count++;

            strcpy(response_pairs[response_count].key, "drive_speed");
            sprintf(response_pairs[response_count].value, "%d",
                    drive_forward ? current_drive_speed : -(int)current_drive_speed);
            response_count++;

            strcpy(response_pairs[response_count].key, "target_drive_speed");
            sprintf(response_pairs[response_count].value, "%d",
                    drive_forward ? target_drive_speed : -(int)target_drive_speed);
            response_count++;

            strcpy(response_pairs[response_count].key, "steering_active");
            strcpy(response_pairs[response_count].value, steering_active ? "true" : "false");
            response_count++;

            strcpy(response_pairs[response_count].key, "target_steering_angle");
            sprintf(response_pairs[response_count].value, "%d", target_steering_angle);
            response_count++;

            strcpy(response_pairs[response_count].key, "current_steering_angle");
            sprintf(response_pairs[response_count].value, "%d",
                    as5600_angl_to_degr(as5600_read_raw_angl(&steering_encoder), 0, 4095)
                    + ENCODER_OFFSET);
            response_count++;

            strcpy(response_pairs[response_count].key, "current_steering_speed");
            sprintf(response_pairs[response_count].value, "%d", current_steering_speed);
            response_count++;
        }
    }

    if (response_count == 0) {
        strcpy(response_pairs[response_count].key,   "result");
        strcpy(response_pairs[response_count].value, "ok");
        response_count++;
    }

    create_json_response(response_str, MAX_JSON_SIZE, response_pairs, response_count);
    DEBUG_PRINT("Sending response: %s\n", response_str);
}


int parse_json(char *json_str, JsonPair *pairs, int max_pairs) {
    char *ptr       = json_str;
    int   pair_count = 0;

    while (*ptr && *ptr != '{') ptr++;
    if (*ptr != '{') return 0;
    ptr++;

    while (*ptr && pair_count < max_pairs) {
        while (*ptr && *ptr != '"') ptr++;
        if (!*ptr) break;
        ptr++;

        char *key_start = pairs[pair_count].key;
        int   key_len   = 0;
        while (*ptr && *ptr != '"' && key_len < MAX_KEY_SIZE - 1) {
            *key_start++ = *ptr++;
            key_len++;
        }
        *key_start = '\0';
        if (!*ptr) break;
        ptr++;

        while (*ptr && *ptr != ':') ptr++;
        if (!*ptr) break;
        ptr++;

        while (*ptr && (*ptr == ' ' || *ptr == '\t')) ptr++;

        if (*ptr == '"') {
            ptr++;
            char *val_start = pairs[pair_count].value;
            int   val_len   = 0;
            while (*ptr && *ptr != '"' && val_len < MAX_VALUE_SIZE - 1) {
                *val_start++ = *ptr++;
                val_len++;
            }
            *val_start = '\0';
            if (!*ptr) break;
            ptr++;
        } else {
            char *val_start = pairs[pair_count].value;
            int   val_len   = 0;
            while (*ptr && *ptr != ',' && *ptr != '}' &&
                   val_len < MAX_VALUE_SIZE - 1 &&
                   *ptr != ' ' && *ptr != '\t') {
                *val_start++ = *ptr++;
                val_len++;
            }
            *val_start = '\0';
        }

        pair_count++;
        while (*ptr && *ptr != ',' && *ptr != '}') ptr++;
        if (!*ptr || *ptr == '}') break;
        ptr++;
    }

    return pair_count;
}


void create_json_response(char *json_str, int max_size, JsonPair *pairs, int pair_count) {
    int offset = 0;
    offset += snprintf(json_str + offset, max_size - offset, "{");

    for (int i = 0; i < pair_count; i++) {
        char   *endptr;
        strtol(pairs[i].value, &endptr, 10);
        bool is_numeric = (*endptr == '\0' && pairs[i].value[0] != '\0');

        if (is_numeric) {
            offset += snprintf(json_str + offset, max_size - offset,
                               "\"%s\": %s", pairs[i].key, pairs[i].value);
        } else {
            offset += snprintf(json_str + offset, max_size - offset,
                               "\"%s\": \"%s\"", pairs[i].key, pairs[i].value);
        }

        if (i < pair_count - 1) {
            offset += snprintf(json_str + offset, max_size - offset, ", ");
        }
    }

    snprintf(json_str + offset, max_size - offset, "}");
}


int32_t process_tcp_server(uint8_t *buf, uint16_t port) {
    int32_t  ret;
    uint16_t size = 0, sentsize = 0;
    uint8_t  destip[4];
    uint16_t destport;
    char     response_buf[MAX_JSON_SIZE];

    uint8_t server_socket  = WHEEL_NUMBER % MAX_SOCKETS;
    uint8_t current_status = getSn_SR(server_socket);

    socket_pool[server_socket].state = current_status;

    static uint8_t prev_status = 0xFF;
    if (prev_status != current_status) {
        if (current_status == SOCK_ESTABLISHED) {
            connection_active    = true;
            last_connection_time = to_ms_since_boot(get_absolute_time());
            socket_pool[server_socket].last_activity = last_connection_time;
            DEBUG_PRINT("Connection established on socket %d\n", server_socket);
            getSn_DIPR(server_socket, destip);
            destport = getSn_DPORT(server_socket);
            DEBUG_PRINT("Remote: %d.%d.%d.%d:%d\n",
                        destip[0], destip[1], destip[2], destip[3], destport);

        } else if (prev_status == SOCK_ESTABLISHED &&
                   (current_status == SOCK_CLOSE_WAIT || current_status == SOCK_CLOSED)) {
            DEBUG_PRINT("Connection lost on socket %d, halting motors\n", server_socket);
            bool any_connected = false;
            for (int i = 0; i < MAX_SOCKETS; i++) {
                if (socket_pool[i].state == SOCK_ESTABLISHED) { any_connected = true; break; }
            }
            if (!any_connected) {
                connection_active = false;
                halt_motors();
            }
        }
        prev_status = current_status;
    }

    switch (current_status) {
        case SOCK_ESTABLISHED:
            if (getSn_IR(server_socket) & Sn_IR_CON) {
                getSn_DIPR(server_socket, destip);
                destport = getSn_DPORT(server_socket);
                DEBUG_PRINT("%d:Connected - %d.%d.%d.%d : %d\r\n",
                            server_socket,
                            destip[0], destip[1], destip[2], destip[3], destport);
                setSn_IR(server_socket, Sn_IR_CON);
            }

            if ((size = getSn_RX_RSR(server_socket)) > 0) {
                socket_pool[server_socket].last_activity = to_ms_since_boot(get_absolute_time());

                // FIX: Leave room for null terminator — receive at most BUF-1 bytes
                if (size > ETHERNET_BUF_MAX_SIZE - 1) size = ETHERNET_BUF_MAX_SIZE - 1;

                ret = recv(server_socket, buf, size);
                if (ret <= 0) {
                    DEBUG_PRINT("Receive error: %d\n", ret);
                    return ret;
                }
                size       = (uint16_t)ret;
                buf[size]  = 0x00;   // safe: size <= ETHERNET_BUF_MAX_SIZE - 1

                process_json_message((char *)buf, response_buf);

                // FIX: Allocate send buffer ONCE outside the send loop
                uint16_t resp_len = (uint16_t)strlen(response_buf);
                char    *send_buf = (char *)malloc(resp_len + 2);
                if (!send_buf) return -1;

                memcpy(send_buf, response_buf, resp_len);
                send_buf[resp_len]     = '\n';
                send_buf[resp_len + 1] = '\0';

                sentsize = 0;
                while (sentsize < resp_len + 1) {
                    ret = send(server_socket,
                               (uint8_t *)(send_buf + sentsize),
                               (resp_len + 1) - sentsize);
                    if (ret < 0) {
                        free(send_buf);
                        close(server_socket);
                        halt_motors();
                        return ret;
                    }
                    sentsize += (uint16_t)ret;
                }
                free(send_buf);
                socket_pool[server_socket].last_activity = to_ms_since_boot(get_absolute_time());
            }
            break;

        case SOCK_CLOSE_WAIT:
            if ((ret = disconnect(server_socket)) != SOCK_OK) return ret;
            DEBUG_PRINT("%d:Socket Closed\r\n", server_socket);
            break;

        case SOCK_INIT:
            DEBUG_PRINT("%d:Listen, JSON TCP server, port [%d]\r\n", server_socket, port);
            if ((ret = listen(server_socket)) != SOCK_OK) return ret;
            break;

        case SOCK_CLOSED:
            DEBUG_PRINT("%d:Socket opening\r\n", server_socket);
            if ((ret = socket(server_socket, Sn_MR_TCP, port, Sn_MR_ND)) != server_socket) {
                DEBUG_PRINT("Socket creation failed: %d\n", ret);
                return ret;
            }
            setSn_KPALVTR(server_socket, 10);
            setSn_CR(server_socket, Sn_CR_SEND_KEEP);
            DEBUG_PRINT("%d:Socket opened\r\n", server_socket);
            break;

        default:
            break;
    }

    return 1;
}


void init_socket_pool(void) {
    for (int i = 0; i < MAX_SOCKETS; i++) {
        socket_pool[i].sock_num      = i;
        socket_pool[i].state         = SOCK_CLOSED;
        socket_pool[i].last_activity = 0;
        socket_pool[i].is_server     = (i == (WHEEL_NUMBER % MAX_SOCKETS));
    }
}

void close_socket_gracefully(uint8_t sock_idx) {
    if (getSn_SR(sock_idx) == SOCK_ESTABLISHED) {
        disconnect(sock_idx);
        uint32_t start = to_ms_since_boot(get_absolute_time());
        while ((to_ms_since_boot(get_absolute_time()) - start < 200) &&
               (getSn_SR(sock_idx) != SOCK_CLOSED)) {
            sleep_ms(10);
        }
    }
    if (getSn_SR(sock_idx) != SOCK_CLOSED) {
        close(sock_idx);
    }
    socket_pool[sock_idx].state = SOCK_CLOSED;
}

void monitor_sockets(void) {
    static uint32_t last_check_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (current_time - last_check_time < CONNECTION_MONITOR_INTERVAL) return;
    last_check_time = current_time;

    for (int i = 0; i < MAX_SOCKETS; i++) {
        uint8_t sock_idx      = socket_pool[i].sock_num;
        uint8_t current_state = getSn_SR(sock_idx);
        socket_pool[i].state  = current_state;

        if (current_state == SOCK_ESTABLISHED) {
            if (current_time - socket_pool[i].last_activity > CONNECTION_TIMEOUT) {
                DEBUG_PRINT("Socket %d timed out, closing\n", sock_idx);
                close_socket_gracefully(sock_idx);
            }
        } else if (current_state == SOCK_CLOSE_WAIT) {
            disconnect(sock_idx);
        }

        if (socket_pool[i].is_server && current_state == SOCK_CLOSED) {
            int ret = socket(sock_idx, Sn_MR_TCP, local_port, Sn_MR_ND);
            if (ret == sock_idx) {
                listen(sock_idx);
                DEBUG_PRINT("Server socket %d reopened on port %d\n", sock_idx, local_port);
            }
        }
    }
}

static void set_clock_khz(void) {
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    clock_configure(
        clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        PLL_SYS_KHZ * 1000,
        PLL_SYS_KHZ * 1000
    );
} bu main c necedi ai kimi durur yoxsa telebe kimi