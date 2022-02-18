#ifndef BUZZ_UTILITY_H
#define BUZZ_UTILITY_H
#include "fc_inav.h"

extern int buzz_listen(const char* type, int msg_size);

extern int buzz_script_set(const char* bo_filename, const char* bdbg_filename,int robot_id);

extern void buzz_script_step();

extern void buzz_script_destroy();

extern int buzz_script_done();

// extern void camera_routine();

// extern void start_blink();

// External parameters
int FREQUENCY;
//int ROBOT_ID;
char* SERVER_ADDR;

#endif
