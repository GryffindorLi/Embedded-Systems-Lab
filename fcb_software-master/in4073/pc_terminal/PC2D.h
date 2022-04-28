#include <inttypes.h>
#include <stdbool.h>

// mode defines
#define MODE_SAFE 0
#define MODE_PANIC 1
#define MODE_MANUAL 2
#define MODE_CALIBRATION 3
#define MODE_YAW_CONTROL 4
#define MODE_FULL_CONTROL 5
#define MODE_RAW 6
#define MODE_HEIGHT_CONTROL 7
#define MODE_WIRELESS 8


// control type define
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} controls;

// the message type
typedef struct {
    uint8_t Preamble; // 1 byte
    uint8_t checksum; // 1 byte
    uint8_t mode;     // 1 byte
    controls control; // 2 * 3 byte
    char key;         // 1 byte
} PC2D_message;

typedef PC2D_message* PC2D_message_p;

// create message from scratch
PC2D_message create_message(void);

// create message from previous message
PC2D_message create_message_from_prev(PC2D_message_p old_message);

// change message contents
void set_checksum(PC2D_message_p mes, uint8_t sum);
void set_mode(PC2D_message_p mes, uint8_t m);
void set_control(PC2D_message_p mes, controls cont);
void set_key(PC2D_message_p mes, char k);
