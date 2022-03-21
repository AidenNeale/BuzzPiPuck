#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "PiPuck.h"

// Default E-Puck I2C bus and Firmware Address
const char I2C_CHANNEL[12] = "/dev/i2c-12";
const char LEGACY_I2C_CHANNEL[11] = "/dev/i2c-4";
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

I2CDevice I2CStruct;

/*
 * Function: i2c_initialise
 * -------------------------
 * Initialises the I2C Struct with I2C Information relevant to the Pi-Puck
 */
void i2c_initialise(void)
{
  memset(&I2CStruct, 0, (sizeof(&I2CStruct)/sizeof(I2CStruct)));

  if (((&I2CStruct)->bus = i2c_open( I2C_CHANNEL )) == -1)
  {
    /* This attempts to open the I2C Channel used on Legacy Kernels */
    if (((&I2CStruct)->bus = i2c_open( LEGACY_I2C_CHANNEL )) == -1)
    {
      printf("I2C Open returns -1");
    }
  }

  (&I2CStruct)->addr = EPUCK_I2C_ADDR;
  (&I2CStruct)->iaddr_bytes = 1;	/* Device internal address is 1 byte */
  (&I2CStruct)->page_bytes = 16; /* Device are capable of 16 bytes per page */

}

/*
 * Function: i2c_destroy
 * ----------------------
 * Closes the I2C Connection
 */
void i2c_destroy()
{
  i2c_close((&I2CStruct)->bus);
}


/*
 * Function: write_data_8
 * -------------------------------------
 * Writes 8 bits of data to the I2C Bus
 *
 * Parameters:
 * --------------------
 * -> address:    The corresponding I2C device address for the operation desired.
 *                Examples of the relevant addresses are found as constants at the top
 *                of the file.
 *
 * -> passedData: Given data to send to I2C 'Address'. This will likely activate/deactivate
 *                whatever is on the given address.
 */
void write_data_8(unsigned int address, u_int8_t passedData)
{
  i2c_write(&I2CStruct, address, &passedData, 1U);
}


/*
 * Function: write_data_16
 * -------------------------------------
 * Writes 16 bits of data to the I2C Bus
 *
 * Parameters:
 * --------------------
 * -> address:    The corresponding I2C device address for the operation desired.
 *                Examples of the relevant addresses are found as constants at the top
 *                of the file.
 *
 * -> passedData: Given data to send to I2C 'Address'. This will likely activate/deactivate
 *                whatever is on the given address.
 */
void write_data_16(unsigned int address, u_int16_t passedData)
{
  i2c_write(&I2CStruct, address, &passedData, 2U);
}


/*
 * Function: read_data_8
 * -------------------------------------
 * Reads 8 bits of data from the I2C Bus
 *
 * Parameters:
 * --------------------
 * -> address:    The corresponding I2C device address for the operation desired.
 *                Examples of the relevant addresses are found as constants at the top
 *                of the file.
 *
 * -> data: A buffer to read the data stored on the I2C 'Address'
 */
void read_data_8(unsigned int address, uint8_t* data)
{
  i2c_read(&I2CStruct, address, data, 1U);
}


/*
 * Function: read_data_16
 * -------------------------------------
 * Reads 16 bits of data from the I2C Bus
 *
 * Parameters:
 * --------------------
 * -> address:    The corresponding I2C device address for the operation desired.
 *                Examples of the relevant addresses are found as constants at the top
 *                of the file.
 *
 * -> data: A buffer to read the data stored on the I2C 'Address'
 */
void read_data_16(unsigned int address, uint16_t* data)
{
  i2c_read(&I2CStruct, address, data, 2U);
}


/*
 * Function: set_outer_leds_byte
 * --------------------------------------------
 * Writes a byte of data to the Pi-Puck's outer LEDs I2C Address
 *
 * Parameters:
 * --------------------
 * -> leds: One-Hot-Encoding for the Outer LEDs of the Pi-Puck
 */
void set_outer_leds_byte(int leds)
{
  write_data_8(OUTER_LEDS, leds);
}


/*
 * Function: set_outer_leds
 * ----------------------
 * Takes input from Buzz Script and equates boolean inputs into an integer for which
 * outer LED to set
 *
 * Parameters:
 * --------------------
 * -> ledX: Boolean indicating whether to enable an LED (1) or disable an LED (0)
 *       where X is are the values 0-7
 */
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


/*
 * Function: set_inner_leds
 * ---------------------------------------
 * Takes input from Buzz Script and equates boolean inputs into an integer for which
 * inner LED to set
 *
 * Parameters:
 * --------------------
 * -> ledX: Boolean indicating whether to enable an LED (1) or disable an LED (0)
 *       where X is are the values 0-7
 */
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


/*
 * Function: set_left_motor_speed
 * ----------------------------------------------------------
 * Sets the left motor speed
 *
 * Parameters:
 * --------------------
 * -> speed: Integer value for Speed
 */
void set_left_motor_speed(int speed)
{
  write_data_16(LEFT_MOTOR_SPEED, speed);
}


/*
 * Function: set_right_motor_speed
 * ----------------------------------------------------------
 * Sets the right motor speed
 *
 * Parameters:
 * --------------------
 * -> speed: Integer value for Speed
 */
void set_right_motor_speed(int speed)
{
  write_data_16(RIGHT_MOTOR_SPEED, speed);
}


/*
 * Function: set_motor_speeds
 * ----------------------
 * Sets both wheel's motor speeds
 *
 * Parameters:
 * --------------------
 * -> speed_left: Integer value for speed of left wheel
 * -> speed_right: Integer value for speed of right wheel
 */
void set_motor_speeds(int speed_left, int speed_right)
{
  write_data_16(LEFT_MOTOR_SPEED, speed_left);
  write_data_16(RIGHT_MOTOR_SPEED, speed_right);
}


/*
 * Function: get_left_motor_speed
 * ----------------------
 * Gets left wheel's motor speeds
 *
 * Returns:
 * --------------------
 * -> data: Unsigned 16 bit Integer value for the motor speed
 */
u_int16_t get_left_motor_speed()
{
  uint16_t data;
  read_data_16(LEFT_MOTOR_SPEED, &data);
  return data;
}


/*
 * Function: get_right_motor_speed
 * ----------------------
 * Gets right wheel's motor speeds
 *
 * Returns:
 * --------------------
 * -> data: Unsigned 16 bit Integer value for the motor speed
 */
u_int16_t get_right_motor_speed()
{
  uint16_t data;
  read_data_16(RIGHT_MOTOR_SPEED, &data);
  return data;
}


/*
 * Function: get_motor_speeds
 * ----------------------
 * Retrieves motor speeds for both wheels
 *
 * Returns:
 * --------------------
 * -> get_left_motor_speed(): Unsigned 16 bit Integer value for the motor speed
 * -> get_right_motor_speed(): Unsigned 16 bit Integer value for the motor speed
 */
u_int16_t get_motor_speeds()
{
  return get_left_motor_speed(), get_right_motor_speed();
}


/*
 * Function: get_left_motor_steps
 * ----------------------
 *
 */
u_int16_t get_left_motor_steps()
{
  uint16_t data;
  read_data_16(LEFT_MOTOR_STEPS, &data);
  return data;
}


/*
 * Function: get_right_motor_steps
 * ----------------------
 *
 */
u_int16_t get_right_motor_steps()
{
  uint16_t data;
  read_data_16(RIGHT_MOTOR_STEPS, &data);
  return data;
}


/*
 * Function: get_motor_steps
 * ----------------------
 *
 */
u_int16_t get_motor_steps()
{
  return get_left_motor_steps(), get_right_motor_steps();
}


/*
 * Function: enable_ir_sensors
 * ----------------------
 * Enables the Infrared Sensors on the Pi-Puck
 *
 * Parameters:
 * --------------------
 * -> enabled: boolean value determining if sensors should be enabled or not
 */
void enable_ir_sensors(bool enabled)
{
  uint8_t data = 0x00;
  if (enabled)
  {
    data += 0x01;
  }
  write_data_8(IR_CONTROL, data);
}


/*
 * Function: get_ir_reflected
 * ----------------------
 * Retrieves the Infrared Sensor values from the Pi-Puck
 *
 * Parameters:
 * --------------------
 * -> sensor: Integer determining which sensor to read from
 *
 * Returns:
 * --------------------
 * -> data: Buffer to read the retrieved data into
 */
u_int16_t get_ir_reflected(int sensor)
{
  uint16_t data;
  read_data_16(IR_REFLECTED_BASE + sensor, &data);
  return data;
}


/*
 * Function: get_ir_ambient
 * ----------------------
 * Retrieves the Infrared Ambient values from the Pi-Puck
 *
 * Parameters:
 * --------------------
 * -> sensor: Integer determining which sensor to read from
 *
 * Returns:
 * --------------------
 * -> data: Buffer to read the retrieved data into
 */
u_int16_t get_ir_ambient(int sensor)
{
  uint16_t data;
  read_data_16(IR_AMBIENT_BASE + sensor, &data);
  return data;
}