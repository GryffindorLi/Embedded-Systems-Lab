#include <stdbool.h>
#include "PC2D.h"

/*
 * @Author: Hanyuan Ban
 */
PC2D_message create_message() {
    PC2D_message new_message = {'*'};
    set_checksum(&new_message, sizeof(new_message));
    return new_message;
}

/*
 * @Author: Hanyuan Ban
 */
PC2D_message create_message_from_prev(PC2D_message_p old_message) {
    PC2D_message new_message = create_message();
    new_message.checksum = old_message->checksum;
    new_message.mode = old_message->mode;
    new_message.control = old_message->control;
    new_message.key = old_message->key;
    return new_message;
}

/*
 * @Author: Hanyuan Ban
 */
void set_checksum(PC2D_message_p mes, uint8_t sum) {
    mes->checksum = sum;
}

/*
 * @Author: Hanyuan Ban
 */
void set_mode(PC2D_message_p mes, uint8_t m) {
    mes->mode = m;
}

/*
 * @Author: Hanyuan Ban
 */
void set_control(PC2D_message_p mes, controls cont) {
    mes->control = cont;
}

/*
 * @Author: Hanyuan Ban
 */
void set_key(PC2D_message_p mes, char k) {
    mes->key = k;
}

