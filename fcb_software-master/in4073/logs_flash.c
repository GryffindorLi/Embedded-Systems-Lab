#include "logs_flash.h"
#include <stdlib.h>
#include "D2PC_drone.h"

static uint32_t curr_addr = 0x00000000;
static bool full = 0;

void write_D2PC_msg_flash(D2PC_message_p msg){

    uint8_t data[100];

    data[0] = (uint8_t)(msg->head);
    data[1] = msg->mode;
    data[2] = highByte(msg->battery);

    data[3] = fourthByte(msg->y);
    data[4] = thirdByte(msg->y);
    data[5] = secondByte(msg->y);
    data[6] = lowByte(msg->y);

    data[7] = fourthByte(msg->p);
    data[8] = thirdByte(msg->p);
    data[9] = secondByte(msg->p);
    data[10] = lowByte(msg->p);

    data[11] = fourthByte(msg->r);
    data[12] = thirdByte(msg->r);
    data[13] = secondByte(msg->r);
    data[14] = lowByte(msg->r);

    data[15] = highByte(msg->filtered_y);
    data[16] = lowByte(msg->filtered_y);

    data[17] = highByte(msg->filtered_p);
    data[18] = lowByte(msg->filtered_p);

    data[19] = highByte(msg->filtered_r);
    data[20] = lowByte(msg->filtered_r);

    data[21] = highByte(msg->motor1);
    data[22] = lowByte(msg->motor1);

    data[23] = highByte(msg->motor2);
    data[24] = lowByte(msg->motor2);

    data[25] = highByte(msg->motor3);
    data[26] = lowByte(msg->motor3);

    data[27] = highByte(msg->motor4);
    data[28] = lowByte(msg->motor4);

    data[29] = highByte(msg->checksum);
    data[30] = lowByte(msg->checksum);

    data[31] = fourthByte(msg->ts);
    data[32] = thirdByte(msg->ts);
    data[33] = secondByte(msg->ts);
    data[34] = lowByte(msg->ts);

    data[35] = lowByte(msg->battery);

    data[36] = (uint8_t)(msg->tail);

    bool res = flash_write_bytes(curr_addr, data, 37);

    if (!res){
        printf("Fail to write to flash!\n");
        return;
    }

    curr_addr += 37;
    if (curr_addr >= 0x01FFFF){
        full = 1;
    }
}

int8_t read_D2PC_msg_flash(D2PC_message_p msg){

    static uint32_t head = 0;

    uint32_t pos = 0x00000000 + head;

    head += 37;
    
    uint8_t data[100];

    //curr_addr -= 33;
    flash_read_bytes(pos, data, 37);

    msg->head = (char)data[0];
    msg->mode = data[1];
    msg->battery = combineByte(data[2], data[35]);

    msg->y = combine32Byte(data[3], data[4], data[5], data[6]);
    msg->p = combine32Byte(data[7], data[8], data[9], data[10]);
    msg->r = combine32Byte(data[11], data[12], data[13], data[14]);

    msg->filtered_y = combineByte(data[15], data[16]);
    msg->filtered_p = combineByte(data[17], data[18]);
    msg->filtered_r = combineByte(data[19], data[20]);

    msg->motor1 = combineByte(data[21], data[22]);
    msg->motor2 = combineByte(data[23], data[24]);
    msg->motor3 = combineByte(data[25], data[26]);
    msg->motor4 = combineByte(data[27], data[28]);

    msg->checksum = combineByte(data[29], data[30]);

    msg->ts = combine32Byte(data[31], data[32], data[33], data[34]);
    
    msg->tail = (char)data[36];
    return 0;
}


uint32_t read_all_from_flash(D2PC_message_p* msgs) {
    uint32_t i = 0;
    int8_t res;
    while ((res = read_D2PC_msg_flash(msgs[i])) != -1) {
        i++;
    }
    return i;
}

void send_all_flash_data(){
    D2PC_message_p* flash_ms = (D2PC_message_p*)malloc(SEND_INTERVAL * sizeof(D2PC_message_p));
	for (uint8_t i = 0; i < SEND_INTERVAL; ++i){
		flash_ms[i] = (D2PC_message_p)malloc(sizeof(D2PC_message));
	}
	uint32_t size = read_all_from_flash(flash_ms);

	for (uint8_t i = 0; i < size; ++i) {
		send_data(flash_ms[i]);
	}

	for (uint8_t i = 0; i < SEND_INTERVAL; ++i) {
		free(flash_ms[i]);
	}

	free(flash_ms);
    flash_chip_erase();
    curr_addr = 0x00000000;
}