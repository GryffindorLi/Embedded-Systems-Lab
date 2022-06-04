#ifndef D2PC_DRONE_H
#define D2PC_DRONE_H

#include <inttypes.h>
#include <stdbool.h>
#include "D2PC.h"

D2PC_message init_message(void);

void delete_message(D2PC_message_p m);

bytes_array to_bytes_array(D2PC_message_p mes);

void delete_bytes_array(bytes_array* b);

D2PC_string_message init_string_message(char s[], uint8_t len);

void delete_string_message(D2PC_string_message_p m);

string_bytes_array to_string_bytes_array(D2PC_string_message_p m);

void delete_string_bytes_array(string_bytes_array* b);

void send_data(D2PC_message_p m);

void print_data(D2PC_message_p p);

#endif