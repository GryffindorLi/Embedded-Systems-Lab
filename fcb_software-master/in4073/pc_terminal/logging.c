#include "../D2PC.h"
#include "logging.h"
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
/*
 * @Author Zirui Li
 * @Param message A pointer to void to accomandate 2 different types of message
 * @Param is_string A int represent whether it is a string message
 */

static int created = 0;
static char File_Name[256];

void logging(void* message, int is_string){

    time_t curr_time = time(NULL);

    if (!created){
        strcpy(File_Name, PARENT_PATH);
        strcat(File_Name, ctime(&curr_time));
        //printf("%s\n", File_Name);
        created = 1;
    }

    FILE* fp = fopen(File_Name, "a");
    if (fp == NULL){
        perror("Open file failed, cannot logging");
        exit(1);
    }

    if (is_string) {
        D2PC_string_message_p sp = (D2PC_string_message_p)message;
        fprintf(fp, "[%s] %s\n", ctime(&curr_time), sp->string);
    } else {
        D2PC_message_p p = (D2PC_message_p)message;
        fprintf(fp, "[%s] mode: %d, battery: %d,"\
            "yaw: %d, pitch: %d, roll: %d, "\
            "filtered_yaw: %d, filtered_pitch: %d, filtered_roll: %d,"\
            " motor1: %d, motor2: %d, motor3: %d, motor4: %d\n", 
            ctime(&curr_time), p->mode, p->battery, p->y, p->p, p->r,
            p->filtered_y, p->filtered_p, p->filtered_r, p->motor1,
            p->motor2, p->motor3, p->motor4);
    }
    fclose(fp);
}