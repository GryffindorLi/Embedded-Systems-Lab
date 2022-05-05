#include <stdbool.h>
#include "D2PC.h"
#include <stdlib.h>
#include <string.h>

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

    while (message.motor != 0){
        sum += (message.motor) & 1;
        message.motor >>= 1;
    }

    return sum;    
}

/*
 * @Author Zirui Li
 * @Return init a message using dynamic allocation, 
 * fill in all field and return the message.
 */
D2PC_message init_message(void){
    D2PC_message m = {.head=HEADER, .mode=0, .battery=19, .y=2, 
                      .r=3, .p=4, .motor=400, .tail=TAIL};

    /* A conceputual implementation
    adc_request_sample();
	read_baro();
    m->mode = get_mode();
    m->battery = get_battery();
    m->y = get_yaw();
    m->r = get_roll();
    m->p = get_pitch();
    m->motor = get_motor();
    */

    m.checksum = cal_checksum(m);

    return m;
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
 * @Return return a pointer to the bytes_array union.
 */
bytes_array to_bytes_array(D2PC_message_p mes){
    bytes_array ba = {.m=*mes};
    return ba;
}

/*
 * @Author Zirui Li
 */
void delete_bytes_array(bytes_array* b){
    //free(b->m);
    free(b);
}

/*
 * @Author Zirui Li
 */
D2PC_string_message init_string_message(void){
    D2PC_string_message sm = {.header=HEADER, .tail=TAIL};
    //sm->string = 
    char* tmp = "This is a debug message, and I will look at it detailly.";
    strcpy(sm.string, tmp);
    return sm;
}

/*
 * @Author Zirui Li
 */
void delete_string_message(D2PC_string_message_p m){
    free(m);
}

/*
 * @Author Zirui Li
 */
string_bytes_array to_string_bytes_array(D2PC_string_message_p m){
    string_bytes_array sba = {.sm=*m};
    return sba;
}

/*
 * @Author Zirui Li
 */
void delete_string_bytes_array(string_bytes_array* b){
    //free(b->sm);
    free(b);
}