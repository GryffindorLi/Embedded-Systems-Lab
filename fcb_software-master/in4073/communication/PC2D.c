#include <stdbool.h>
#include "PC2D.h"

CTRL_msg new_ctrl_msg() {
    CTRL_msg new_message = {'C', {0, 0, 0, 0}, '\0', 0};
    new_message.checksum = sizeof(new_message);
    return new_message;
}

MODE_msg new_mode_msg() {
    MODE_msg new_message = {'M', MODE_SAFE, 0};
    new_message.checksum = sizeof(new_message);
    return new_message;
}

