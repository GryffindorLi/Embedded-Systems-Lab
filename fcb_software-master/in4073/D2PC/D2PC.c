#include <stdbool.h>
#include "D2PC.h"

/*
 * @Author Zirui Li
 * @Param message The message with all fields being filled except the checksum field.
 * @Return the checksum of the message.
 */
uint16_t cal_checksum(D2PC_message message){
    uint16_t sum = 0;

    while (message.mode != 0){
        sum += (message.mode) & 1;
        message.mode >>= 1;
    }

    while (message.battery != 0){
        sum += (message.battery) & 1;
        message.battery >>= 1;
    }

    while (message.y != 0){
        sum += (message.y) & 1;
        message.y >>= 1;
    }

    while (message.p != 0){
        sum += (message.p) & 1;
        message.p >>= 1;
    }

    while (message.r != 0){
        sum += (message.r) & 1;
        message.r >>= 1;
    }

    return sum;    
}

/*
 * @Author Zirui Li
 * @Return init a message using dynamic allocation, 
 * fill in all field and return the message.
 */
D2PC_message init_message(void){
    D2PC_message_p m = (D2PC_message_p)malloc(sizeof(D2PC_message));

    /* A conceputual implementation
    m->mode = get_mode();
    m->battery = get_battery();
    m->y = get_yaw();
    m->r = get_roll();
    m->p = get_pitch();
    */

    m->mode = 0;
    m->battery = 1;
    m->y = 2;
    m->r = 3;
    m->p = 4;

    m->checksum = cal_checksum(*m);

    return *m;
}

/*
 * @Author Zirui Li
*/
void delete_message(D2PC_message_p m){
    free(m);
}

/*
 * @Author Zirui Li
 * @Param m A pointer to a D2PC_message struct.
 * @Param arr A double pointer that points to the pointer of a byte array
 * @Return return the number of bytes, in our current implementation is 10.
 */
int to_bytes_array(D2PC_message_p m, uint16_t** arr){
    bytes_array* ba = (bytes_array*)malloc(sizeof(bytes_array));
    ba->m = *m;
    *arr = ba->bytes;
    return 10;
}