#ifndef D2PC_H
#define D2PC_H

#include <inttypes.h>
#include <stdbool.h>

#define LOG_FROM_TERMINAL
#define DATA_HEADER '@'
#define STRING_HEADER '#'
#define TAIL ';'

#define lowByte(MSG)    ((uint8_t)((MSG) & 0xFF))
#define highByte(MSG)   ((uint8_t)((MSG) >> 8))
#define secondByte(MSG)  ((uint8_t)(((MSG) >> 8) & 0xFF))
#define thirdByte(MSG)  ((uint8_t)(((MSG) >> 16) & 0xFF))
#define fourthByte(MSG)  ((uint8_t)(((MSG) >> 24) & 0xFF))
#define combineByte(MSB,LSB) ((int16_t) (((MSB) << 8) | (LSB)))
#define combine32Byte(MSB1, MSB2, MSB3, LSB) ((int32_t) ((MSB1 << 24) | (MSB2 << 16) | (MSB3 << 8) | LSB))
#define combine32UByte(MSB1, MSB2, MSB3, LSB) ((uint32_t) ((MSB1 << 24) | (MSB2 << 16) | (MSB3 << 8) | LSB))


/*
 * @Author: Zirui Li
 */
typedef struct {
    char head;  // 1 byte 0
    uint8_t mode; // 1 byte 1
    int32_t y;  // 2 bytes 3 4 5 6
    int32_t p;  // 2 bytes 7 8 9 10
    int32_t r;  // 2 bytes 11 12 13 14
    int16_t filtered_y;  // 2 bytes 15 16
    int16_t filtered_p;  // 2 bytes 17 18
    int16_t filtered_r;  // 2 bytes 19 20
    int16_t motor1;  // 2 bytes 21 22
    int16_t motor2; // 23 24
    int16_t motor3;  // 25 26
    int16_t motor4;  // 27 28
    int16_t checksum;  // 2 bytes 29 30
    uint32_t ts;    // 4 bytes 31 32 33 34
    uint16_t battery; // 1 byte 35
    char tail;  // 1 byte 36
} D2PC_message;  //37 bytes
/*
 * @Author: Zirui Li
 */
typedef struct {
    char header; //1 bytes
    char string[255];  // 255 bytes
    char tail;  // 1 bytes
} D2PC_string_message;   // 257 bytes

typedef D2PC_string_message* D2PC_string_message_p;

typedef D2PC_message* D2PC_message_p;

/*
 * @Author: Zirui Li
 */
typedef union {
    D2PC_message m;
    uint8_t bytes[16];
} bytes_array;

/*
 * @Author: Zirui Li
 */
typedef union {
    D2PC_string_message sm;
    uint8_t bytes[257];
} string_bytes_array;

int16_t cal_checksum(D2PC_message message);


#endif