#ifndef BUZZKH4_CLOSURES_H
#define BUZZKH4_CLOSURES_H

#include "PiPuck.h"
#include <buzz/buzzvm.h>

extern int buzz_pipuck_print(buzzvm_t vm);

/*
 * set_wheels(ls,rs) function in Buzz
 * Sets the wheel speeds to ls (left) and rs (right)
 * speeds are expressed in cm/s
 */
extern int pipuck_set_wheels(buzzvm_t vm);

extern int pipuck_set_outer_leds(buzzvm_t vm);

extern int buzz_sleep_ms(buzzvm_t vm);

/*
 * Updates battery information in Buzz
 */
extern int buzzkh4_update_battery(buzzvm_t vm);

/*
 * Updates IR information in Buzz
 * Proximity and ground sensors
 */
extern int buzzkh4_update_ir(buzzvm_t vm);

extern int buzzkh4_update_ir_filtered(buzzvm_t vm);

extern int buzzkh4_update_us(buzzvm_t vm);

extern int buzzkh4_camera_updateblob(buzzvm_t vm, int* blob);

extern int buzzkh4_abs_position(buzzvm_t vm, float x, float y, float theta);

extern int buzzkh4_enable_us(buzzvm_t vm, int value);

extern int buzzkh4_play_sound(buzzvm_t vm, char* filename, int volume);

extern int BuzzGoTo(buzzvm_t vm);

extern buzzvm_state TablePutI(buzzobj_t t_table, const char* str_key, int n_value, buzzvm_t m_tBuzzVM);

extern buzzvm_state TablePutO(buzzobj_t t_table, int n_idx, buzzobj_t t_obj, buzzvm_t m_tBuzzVM);

extern buzzvm_state TablePutF(buzzobj_t t_table, const char* str_key, float n_value, buzzvm_t m_tBuzzVM);

#endif
