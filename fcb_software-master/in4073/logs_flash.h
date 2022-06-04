#include <inttypes.h>
#include <stdbool.h>
#include "D2PC.h"
#include "spi_flash.h"
#include <stdio.h>

#define SEND_INTERVAL 5

void write_D2PC_msg_flash(D2PC_message_p msg);

int8_t read_D2PC_msg_flash(D2PC_message_p msg);

uint32_t read_all_from_flash(D2PC_message_p* msgs);

void send_all_flash_data();