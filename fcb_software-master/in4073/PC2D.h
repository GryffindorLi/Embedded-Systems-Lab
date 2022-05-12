#ifndef __PC2D__
#define __PC2D__

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
    uint16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} controls;

// the message type
typedef struct {
    char h1;      // 1 byte
    char h2;
    controls control; // 2 * 4 byte
    char key;         // 1 byte
    uint8_t checksum; // 1 byte
} CTRL_msg;

typedef struct {
    char h1;      // 1 byte
    char h2;
    uint8_t mode;   // 1 byte
} MODE_msg;

typedef CTRL_msg* CTRL_msg_p;
typedef MODE_msg* MODE_msg_p;

// create message from scratch
CTRL_msg new_ctrl_msg(void);
MODE_msg new_mode_msg(void);
uint16_t safeuint16pint16(uint16_t a, int16_t b);
int16_t safeint16pint16(int16_t a, int16_t b);

#endif
