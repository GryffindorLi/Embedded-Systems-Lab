#include "D2PC_drone.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "timers.h"
#include "uart.h"
#include "control.h"

//static uint8_t curr_idx = 0;
extern uint8_t current_mode;
extern int32_t yaw, pitch, roll;
extern int16_t* aes;
extern uint16_t bat_volt;
extern int16_t phi, theta, psi;

/*
 * @Author Zirui Li
 * @Return init a message using dynamic allocation, 
 * fill in all field and return the message.
 */
D2PC_message init_message(void){
    D2PC_message m = {.head=DATA_HEADER, 
                      .mode=current_mode,
                      .battery=bat_volt, 
                      .y=yaw, 
                      .r=roll,
                      .p=pitch,
                      .filtered_y=phi,
                      .filtered_r=psi,
                      .filtered_p=theta,
                      .motor1=aes[0], 
                      .motor2=aes[1],
                      .motor3=aes[2],
                      .motor4=aes[3],
                      .tail=TAIL,
                      .ts=get_time_us()};

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
 * @Param s A pointer to a char array
 * @Param len The length of the string
 * Init the message using string s.
 */
D2PC_string_message init_string_message(char* s, uint8_t len){
    D2PC_string_message sm = {.header=STRING_HEADER, .tail=TAIL};
    len -= 1;
    //char* tmp = "This is a debug message, and I will look at it detailly.";
    strcpy(sm.string, s);
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
/*
 * @Author Zirui Li
 * @Param m A pointer to a D2PC_message
 * Sending each field byte by byte. This is because C pad the struct, 
 * making the memory layout unreliable and thus cannot directly mapping 
 * from bytes array to struct.
 */
void send_data(D2PC_message_p m) {
    uart_put((uint8_t)(m->head));

	uart_put(m->mode);

	uart_put(fourthByte(m->y));
	uart_put(thirdByte(m->y));
	uart_put(secondByte(m->y));
	uart_put(lowByte(m->y));

	uart_put(fourthByte(m->p));
	uart_put(thirdByte(m->p));
	uart_put(secondByte(m->p));
	uart_put(lowByte(m->p));

	uart_put(fourthByte(m->r));
	uart_put(thirdByte(m->r));
	uart_put(secondByte(m->r));
	uart_put(lowByte(m->r));
	
	uart_put(highByte(m->filtered_y));
	uart_put(lowByte(m->filtered_y));

	uart_put(highByte(m->filtered_p));
	uart_put(lowByte(m->filtered_p));

	uart_put(highByte(m->filtered_r));
	uart_put(lowByte(m->filtered_r));

	uart_put(highByte(m->motor1));
	uart_put(lowByte(m->motor1));
	uart_put(highByte(m->motor2));
	uart_put(lowByte(m->motor2));
	uart_put(highByte(m->motor3));
	uart_put(lowByte(m->motor3));
	uart_put(highByte(m->motor4));
	uart_put(lowByte(m->motor4));
	uart_put(highByte(m->checksum));
	uart_put(lowByte(m->checksum));
    
    
    uart_put(fourthByte(m->ts));
	uart_put(thirdByte(m->ts));
	uart_put(secondByte(m->ts));
	uart_put(lowByte(m->ts));
	uart_put((uint8_t)(m->tail));
}