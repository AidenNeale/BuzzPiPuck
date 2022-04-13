#ifndef BUZZ_UTILITY_H
#define BUZZ_UTILITY_H

extern int buzz_listen(const char* type, int msg_size, int RID);

extern int buzz_script_set(const char* bo_filename, const char* bdbg_filename,int robot_id);

extern void buzz_script_step();

extern void buzz_script_destroy();

extern int buzz_script_done();

extern int DONE;

// External parameters
int FREQUENCY;
//int ROBOT_ID;
char* SERVER_ADDR;

#endif
