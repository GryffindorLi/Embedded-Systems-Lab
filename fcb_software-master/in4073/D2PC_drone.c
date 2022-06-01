#include "D2PC.h"
#include "D2PC_drone.h"
#include "in4073.h"
#include <stdlib.h>
#include <string.h>

extern uint8_t current_mode;
extern int16_t* aes;
extern int16_t phi, theta, psi;


/*
 * @Author Zirui Li
 * @Return init a message using dynamic allocation, 
 * fill in all field and return the message.
 */
D2PC_message init_message(void){
    D2PC_message m = {.head=DATA_HEADER, 
                      .mode=current_mode,
                      .battery=19, 
                      .y=yaw, 
                      .r=roll,
                      .p=pitch,
                      .filtered_y=psi,
                      .filtered_r=phi,
                      .filtered_p=theta,
                      .motor1=aes[0], 
                      .motor2=aes[1],
                      .motor3=aes[2],
                      .motor4=aes[3],
                      .tail=TAIL};

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
    D2PC_string_message sm = {.header=STRING_HEADER, .tail=TAIL};
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