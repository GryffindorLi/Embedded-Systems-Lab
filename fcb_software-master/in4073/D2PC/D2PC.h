#include <inttypes.h>
#include <stdbool.h>

/*
 * @Author: Zirui Li
 */
typedef struct {
    uint8_t mode; // 1 byte
    uint8_t battery; // 1 byte
    uint16_t y;  // 2 bytes
    uint16_t p;  // 2 bytes
    uint16_t r;  // 2 bytes

    uint16_t checksum;  // 2 bytes
} D2PC_message;  //10 bytes

typedef D2PC_message* D2PC_message_p;

/*
 * @Author: Zirui Li
 */
typedef union {
    D2PC_message m;
    uint8_t bytes[10];
} bytes_array;

uint16_t cal_checksum(D2PC_message message);

D2PC_message init_message(void);

void delete_message(D2PC_message_p m);

bytes_array* to_bytes_array(D2PC_message_p m);

void delete_bytes_array(bytes_array* b);