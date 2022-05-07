#ifndef D2PC_H
#define D2PC_H

#include <inttypes.h>
#include <stdbool.h>

#define HEADER '@'
#define TAIL ';'

/*
 * @Author: Zirui Li
 */
typedef struct {
    char head;  // 1 byte
    uint8_t mode; // 1 byte
    uint8_t battery; // 1 byte
    int16_t y;  // 2 bytes
    int16_t p;  // 2 bytes
    int16_t r;  // 2 bytes
    uint16_t motor;  // 2 bytes

    uint16_t checksum;  // 2 bytes
    char tail;  // 1 byte
} D2PC_message;  //12 bytes

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

uint16_t cal_checksum(D2PC_message message);

D2PC_message init_message(void);

void delete_message(D2PC_message_p m);

bytes_array to_bytes_array(D2PC_message_p m);

void delete_bytes_array(bytes_array* b);

D2PC_string_message init_string_message(void);

void delete_string_message(D2PC_string_message_p m);

string_bytes_array to_string_bytes_array(D2PC_string_message_p m);

void delete_string_bytes_array(string_bytes_array* b);

#endif