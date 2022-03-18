#ifndef PIPUCK_H
#define PIPUCK_H

#include <stdlib.h>
#include <stdbool.h>
#include "i2c.h"


// This is how to define a struct
// typedef struct
// {
//     int bus;
//     int controller;
// } I2CData;

// Example Functions
// int ConnectToBus(void);
// int SelectMotor(int motor);
// int MoveMotor(int amount);



void initialise(void);
void _cleanUp(I2CDevice*);

void write_data_8(unsigned int, u_int8_t);
void write_data_16(unsigned int, u_int16_t);
void read_data_8(unsigned int, u_int8_t*);
void read_data_16(unsigned int, u_int16_t*);

void set_outer_leds_byte(int);
void set_outer_leds(bool, bool, bool, bool, bool, bool, bool, bool);
void set_inner_leds(bool, bool);

void set_left_motor_speed(int);
void set_right_motor_speed(int);
void set_motor_speeds(int, int);

u_int16_t get_left_motor_speed(void);
u_int16_t get_right_motor_speed(void);
u_int16_t get_motor_speeds(void);

u_int16_t get_left_motor_steps(void);
u_int16_t get_right_motor_steps(void);
u_int16_t get_motor_steps(void);

void enable_ir_sensors(bool);
u_int16_t get_ir_reflected(int);
u_int16_t get_ir_ambient(int);

#endif /* PIPUCK_H */