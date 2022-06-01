#include "D2PC.h"

D2PC_message init_message(void);

void delete_message(D2PC_message_p m);

bytes_array to_bytes_array(D2PC_message_p m);

void delete_bytes_array(bytes_array* b);

D2PC_string_message init_string_message(void);

void delete_string_message(D2PC_string_message_p m);

string_bytes_array to_string_bytes_array(D2PC_string_message_p m);

void delete_string_bytes_array(string_bytes_array* b);