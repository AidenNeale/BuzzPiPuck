#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "PiPuck.h"

// Default E-Puck I2C bus and Firmware Address
const int I2C_CHANNEL = 12;
const int LEGACY_I2C_CHANNEL = 4;
const int EPUCK_I2C_ADDR = 0x1E;

// Register addresses
const int OUTER_LEDS = 0;
const int INNER_LEDS = 1;
const int LEFT_MOTOR_SPEED = 2;
const int RIGHT_MOTOR_SPEED = 3;
const int LEFT_MOTOR_STEPS = 4;
const int RIGHT_MOTOR_STEPS = 5;
const int IR_CONTROL = 6;
const int IR_REFLECTED_BASE = 7;
const int IR0_REFLECTED = 7;
const int IR1_REFLECTED = 8;
const int IR2_REFLECTED = 9;
const int IR3_REFLECTED = 10;
const int IR4_REFLECTED = 11;
const int IR5_REFLECTED = 12;
const int IR6_REFLECTED = 13;
const int IR7_REFLECTED = 14;
const int IR_AMBIENT_BASE = 15;
const int IR0_AMBIENT = 15;
const int IR1_AMBIENT = 16;
const int IR2_AMBIENT = 17;
const int IR3_AMBIENT = 18;
const int IR4_AMBIENT = 19;
const int IR5_AMBIENT = 20;
const int IR6_AMBIENT = 21;
const int IR7_AMBIENT = 22;


void initialise(I2CDevice * I2CStruct)
{
  memset(I2CStruct, 0, (sizeof(&I2CStruct)/sizeof(I2CStruct)));

  if ((I2CStruct->bus = i2c_open("/dev/i2c-4")) == -1)
  {
    printf("I2C Open returns -1");
  }

  I2CStruct->addr = EPUCK_I2C_ADDR;
  I2CStruct->iaddr_bytes = 1;	/* Device internal address is 1 byte */
  I2CStruct->page_bytes = 16; /* Device are capable of 16 bytes per page */

}

void _cleanUp(I2CDevice* I2CStruct)
{
  i2c_close(*(&I2CStruct->bus));
}

void write_data_8(unsigned int address, u_int8_t passedData)
{
  I2CDevice I2CStruct;
  initialise(&I2CStruct);

  i2c_write(&I2CStruct, address, &passedData, 1U);

  _cleanUp(&I2CStruct);
}


void write_data_16(unsigned int address, u_int16_t passedData)
{
  I2CDevice I2CStruct;
  initialise(&I2CStruct);

  i2c_write(&I2CStruct, address, &passedData, 2U);

  _cleanUp(&I2CStruct);
}


void read_data_8(unsigned int address, uint8_t* data)
{
  I2CDevice I2CStruct;
  initialise(&I2CStruct);
  i2c_read(&I2CStruct, address, data, 1U);
  _cleanUp(&I2CStruct);
}


void read_data_16(unsigned int address, uint16_t* data)
{
  I2CDevice I2CStruct;
  initialise(&I2CStruct);
  i2c_read(&I2CStruct, address, data, 2U);
  _cleanUp(&I2CStruct);
}

void set_outer_leds_byte(int leds)
{
  write_data_8(OUTER_LEDS, leds);
}


void set_outer_leds(bool led0, bool led1, bool led2, bool led3, bool led4, bool led5, bool led6, bool led7)
{
  int data = 0x00;
  if (led0)
  {
    data += 0x01;
  }
  if (led1)
  {
    data += 0x02;
  }
  if (led2)
  {
    data += 0x04;
  }
  if (led3)
  {
    data += 0x08;
  }
  if (led4)
  {
    data += 0x10;
  }
  if (led5)
  {
    data += 0x20;
  }
  if (led6)
  {
    data += 0x40;
  }
  if (led7)
  {
    data += 0x80;
  }
  
  set_outer_leds_byte(data);
}


void set_inner_leds(bool front, bool body)
{
  int data = 0x00;
  if (front)
  {
    data += 0x01;
  }
  if (body)
  {
    data += 0x02;
  }

  write_data_8(INNER_LEDS, data);
}


void set_left_motor_speed(int speed)
{
  write_data_16(LEFT_MOTOR_SPEED, speed);
}


void set_right_motor_speed(int speed)
{
  write_data_16(RIGHT_MOTOR_SPEED, speed);
}


void set_motor_speeds(int speed_left, int speed_right)
{
  write_data_16(LEFT_MOTOR_SPEED, speed_left);
  write_data_16(RIGHT_MOTOR_SPEED, speed_right);
}


u_int16_t get_left_motor_speed()
{
  uint16_t data;
  read_data_16(LEFT_MOTOR_SPEED, &data);
  return data;
}


u_int16_t get_right_motor_speed()
{
  uint16_t data;
  read_data_16(RIGHT_MOTOR_SPEED, &data);
  return data;
}


u_int16_t get_motor_speeds()
{
  return get_left_motor_speed(), get_right_motor_speed();
}


u_int16_t get_left_motor_steps()
{
  uint16_t data;
  read_data_16(LEFT_MOTOR_STEPS, &data);
  return data;
}


u_int16_t get_right_motor_steps()
{
  uint16_t data;
  read_data_16(RIGHT_MOTOR_STEPS, &data);
  return data;
}


u_int16_t get_motor_steps()
{
  return get_left_motor_steps(), get_right_motor_steps();
}


void enable_ir_sensors(bool enabled)
{
  uint8_t data = 0x00;
  if (enabled)
  {
    data += 0x01;
  }
  write_data_8(IR_CONTROL, data);
}


u_int16_t get_ir_reflected(int sensor)
{
  uint16_t data;
  read_data_16(IR_REFLECTED_BASE + sensor, &data);
  return data;
}


u_int16_t get_ir_ambient(int sensor)
{
  uint16_t data;
  read_data_16(IR_AMBIENT_BASE + sensor, &data);
  return data;
}