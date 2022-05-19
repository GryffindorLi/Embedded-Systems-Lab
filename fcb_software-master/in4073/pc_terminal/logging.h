#ifndef LOGGING_H_
#define LOGGING_H_

#include "../D2PC.h"
#include "logging.h"
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

/*
#define PARENT_PATH "/home/ubuntu/Desktop/"\
    "CS4140ES-Embedded-Systems-Lab"\
    "/fcb_software-master/in4073/pc_terminal/logs/log_"
*/
//time_t curr_time = time(NULL);
//strcat(parent, ctime(&curr_time));

#define PARENT_PATH "./logs/log_"

void logging(void* message, int is_string);

#endif