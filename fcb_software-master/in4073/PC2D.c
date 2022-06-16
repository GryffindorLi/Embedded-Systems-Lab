#include <stdbool.h>
#include "PC2D.h"

/*
 * @Author Hanyuan Ban
 * @Param None
 * @Return initiated ctrl msg
 */
CTRL_msg new_ctrl_msg() {
    CTRL_msg new_message = {'C', 't', {0, 0, 0, 0, 0}, '\0', 0};
    new_message.checksum = sizeof(new_message);
    return new_message;
}

/*
 * @Author Hanyuan Ban
 * @Param None
 * @Return initiated mode msg
 */
MODE_msg new_mode_msg() {
    MODE_msg new_message = {'M', 'd', MODE_SAFE};
    return new_message;
}