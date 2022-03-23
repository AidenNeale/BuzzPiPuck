#define _GNU_SOURCE
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <stdio.h>
#include "buzzpipuck_closures.h"
#include "pipuck_utility.h"

float POSE[4] = {0, 0, 0, 0}; //X, Y, Z, Theta
/****************************************/
/****************************************/

int buzz_pipuck_print(buzzvm_t vm) {
   int i;
   for(i = 1; i < buzzdarray_size(vm->lsyms->syms); ++i) {
      buzzvm_lload(vm, i);
      buzzobj_t o = buzzvm_stack_at(vm, 1);
      buzzvm_pop(vm);
      switch(o->o.type) {
         case BUZZTYPE_NIL:
            fprintf(stdout, "[nil]");
            break;
         case BUZZTYPE_INT:
            fprintf(stdout, "%d", o->i.value);
            break;
         case BUZZTYPE_FLOAT:
            fprintf(stdout, "%f", o->f.value);
            break;
         case BUZZTYPE_TABLE:
            fprintf(stdout, "[table with %d elems]", (buzzdict_size(o->t.value)));
            break;
         case BUZZTYPE_CLOSURE:
            if(o->c.value.isnative)
               fprintf(stdout, "[n-closure @%d]", o->c.value.ref);
            else
               fprintf(stdout, "[c-closure @%d]", o->c.value.ref);
            break;
         case BUZZTYPE_STRING:
            fprintf(stdout, "%s", o->s.value.str);
            break;
         case BUZZTYPE_USERDATA:
            fprintf(stdout, "[userdata @%p]", o->u.value);
            break;
         default:
            break;
      }
   }
   fprintf(stdout, "\n");
   return buzzvm_ret0(vm);
}

int pipuck_set_wheels(buzzvm_t vm) {
   buzzvm_lnum_assert(vm, 2);
   buzzvm_lload(vm, 1); /* Left speed */
   buzzvm_lload(vm, 2); /* Right speed */
   buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
   set_motor_speeds(buzzvm_stack_at(vm, 2)->f.value * 10.0f, /* Left speed */
                    buzzvm_stack_at(vm, 1)->f.value * 10.0f);/* Right speed */
   return buzzvm_ret0(vm);
}

int pipuck_set_outer_leds(buzzvm_t vm) {
  buzzvm_lnum_assert(vm, 8);
  buzzvm_lload(vm, 1); /* LED0 */
  buzzvm_lload(vm, 2); /* LED1 */
  buzzvm_lload(vm, 3); /* LED2 */
  buzzvm_lload(vm, 4); /* LED3 */
  buzzvm_lload(vm, 5); /* LED4 */
  buzzvm_lload(vm, 6); /* LED5 */
  buzzvm_lload(vm, 7); /* LED6 */
  buzzvm_lload(vm, 8); /* LED7 */
  buzzvm_type_assert(vm, 8, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 7, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 6, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 5, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 4, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
  set_outer_leds(buzzvm_stack_at(vm, 1)->i.value,  /* LED0 */
                buzzvm_stack_at(vm, 2)->i.value,  /* LED1 */
                buzzvm_stack_at(vm, 3)->i.value,  /* LED2 */
                buzzvm_stack_at(vm, 4)->i.value,  /* LED3 */
                buzzvm_stack_at(vm, 5)->i.value,  /* LED4 */
                buzzvm_stack_at(vm, 6)->i.value,  /* LED5 */
                buzzvm_stack_at(vm, 7)->i.value,  /* LED6 */
                buzzvm_stack_at(vm, 8)->i.value); /* LED7 */
  return buzzvm_ret0(vm);
}

void WrapValue(float *t_value) {
         while(*t_value > 3.1416) *t_value -= 2*3.1416;
         while(*t_value < -3.1416) *t_value += 2*3.1416;
}

float calculate_rel_theta(float *vect) {

  float angle = atan2(vect[1],vect[0]);
  WrapValue(&angle);

  return angle;
}

float calculate_rel_distance(float* vect) {
   /* Get the length of the heading vector */
   float fHeadingLength = pow(vect[0], 2) + pow(vect[1], 2);
   fHeadingLength = sqrt(fHeadingLength);
   return fHeadingLength;
}

int pipuck_goto(buzzvm_t vm) {
  printf("X: %f, Y: %f, Z: %f, Theta: %f", POSE[0], POSE[1], POSE[2], POSE[3]);
  /*Retrieves X and Y Coordinates of Goto */
  buzzvm_lnum_assert(vm, 2);
  buzzvm_lload(vm, 1); /* Y Coordinate */
  buzzvm_lload(vm, 2); /* X Coordinate */
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);

  float gotoX = buzzvm_stack_at(vm, 2)->f.value;
  float gotoY = buzzvm_stack_at(vm, 1)->f.value;

  /* Create a vector relative to the position of the robot */
  float vect[2];
  vect[0]= gotoX - POSE[0];
  vect[1]= gotoY - POSE[1];

  float angle = calculate_rel_theta(vect);
  // float distance = calculate_rel_distance(vect);

  while (angle < 10 && angle > -10) {
    vect[0]= gotoX - POSE[0];
    vect[1]= gotoY - POSE[1];
    angle = calculate_rel_theta(vect);
    if (angle > 10) {
      set_motor_speeds(20, 50);
    }
    if (angle < -10) {
      set_motor_speeds(50, 20);
    }
    else{
      printf("Vector X: %f, Vector Y: %f\n\r", vect[0], vect[1]);
      printf("Pose[0]: %f, Pose[1]: %f\n\r", POSE[0], POSE[1]);
      printf("Calculated Angle: %f\n\r", angle);
    }
  }

  return buzzvm_ret0(vm);
}


int buzz_sleep_ms(buzzvm_t vm) {
  buzzvm_lnum_assert(vm, 1);
  buzzvm_lload(vm, 1);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);

  struct timespec ts;
  ts.tv_sec = ((int) buzzvm_stack_at(vm, 1)->f.value) / 1;
  ts.tv_nsec = (((int) buzzvm_stack_at(vm, 1)->f.value) % 1) * 1000000000;
  nanosleep(&ts, NULL);

  return buzzvm_ret0(vm);
}


// void SetWheelSpeedsFromVector(float* vec) {
//    float HardTurnOnAngleThreshold = 1.57; //90.0 deg
//    float SoftTurnOnAngleThreshold = 1.2217; //70.0 deg
//    float NoTurnAngleThreshold = 0.1745; //10.0 deg
//    float MaxSpeed = 20.0;
// //printf("Got (%.2f,%.2f), turning is %i\n", vec[0], vec[1], TurningMechanism);
//    /* Get the heading angle */
//    float cHeadingAngle = atan2 (vec[1],vec[0]);
//    WrapValue(&cHeadingAngle);
//    /* Get the length of the heading vector */
//    float fHeadingLength = vec[0]*vec[0]+vec[1]*vec[1];
//    fHeadingLength = sqrt(fHeadingLength);
// //printf("Compute distance %.2f and angle %.2f\n", fHeadingLength, cHeadingAngle);
//    /* Clamp the speed so that it's not greater than MaxSpeed */
//    float fBaseAngularWheelSpeed = MIN((float)fHeadingLength, (float)MaxSpeed);
// //printf("fBaseAngularWheelSpeed = %.2f\n", fBaseAngularWheelSpeed);

//    /* Turning state switching conditions */
//    if(fabs(cHeadingAngle) <= NoTurnAngleThreshold) {
//       /* No Turn, heading angle very small */
//       TurningMechanism = 0;
//    }
//    else if(fabs(cHeadingAngle) > HardTurnOnAngleThreshold) {
//       /* Hard Turn, heading angle very large */
//       TurningMechanism = 2;
//    }
//    else if(TurningMechanism == 0 &&
//            fabs(cHeadingAngle) > SoftTurnOnAngleThreshold) {
//       /* Soft Turn, heading angle in between the two cases */
//       TurningMechanism = 1;
//    }

//    /* Wheel speeds based on current turning state */
//    float fSpeed1 = 0 , fSpeed2 = 0;
//    switch(TurningMechanism) {
//       case 0: {
//          /* Just go straight */
//          fSpeed1 = fBaseAngularWheelSpeed;
//          fSpeed2 = fBaseAngularWheelSpeed;
//          break;
//       }

//       case 1: { //soft turn
//          /* Both wheels go straight, but one is faster than the other */
//          float fSpeedFactor = (HardTurnOnAngleThreshold - abs(cHeadingAngle)) / HardTurnOnAngleThreshold;
//          fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
//          fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
//          break;
//       }

//       case 2: { //hard turn
//          /* Opposite wheel speeds */
//          fSpeed1 = -MaxSpeed;
//          fSpeed2 =  MaxSpeed;
//          break;
//       }
//    }

//    /* Apply the calculated speeds to the appropriate wheels */
//    float fLeftWheelSpeed, fRightWheelSpeed;
//    if(cHeadingAngle > 0) {
//       /* Turn Left */
//       fLeftWheelSpeed  = fSpeed1;
//       fRightWheelSpeed = fSpeed2;
//    }
//    else {
//       /* Turn Right */
//       fLeftWheelSpeed  = fSpeed2;
//       fRightWheelSpeed = fSpeed1;
//    }
//    /* Finally, set the wheel speeds */
// //printf("Sending %.2f - %.2f to the wheels (%i)\n",fLeftWheelSpeed, fRightWheelSpeed, TurningMechanism);
//   //kh4_set_speed(fLeftWheelSpeed, fRightWheelSpeed,DSPIC);
// }
// /****************************************/
// int BuzzGoTo(buzzvm_t vm) {
//    buzzvm_lnum_assert(vm, 2);
//    /* Push the vector components */
//    buzzvm_lload(vm, 1);
//    buzzvm_lload(vm, 2);
//    buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
//    buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
//    /* Create a new vector with that */
//    float vect[2];
//    vect[0]=buzzvm_stack_at(vm, 2)->f.value;
//    vect[1]=buzzvm_stack_at(vm, 1)->f.value;
//    SetWheelSpeedsFromVector(vect);
//    return buzzvm_ret0(vm);
// }

/****************************************/
/****************************************/

// int buzzkh4_update_battery(buzzvm_t vm) {
//    static char BATTERY_BUF[256];
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "battery", 1));
//    buzzvm_pusht(vm);
//    buzzvm_dup(vm);
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "voltage", 1));
//    buzzvm_pushf(vm, (BATTERY_BUF[10] | BATTERY_BUF[11] << 8) * .00975f);
//    buzzvm_tput(vm);
//    buzzvm_dup(vm);
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "current", 1));
//    buzzvm_pushf(vm, (BATTERY_BUF[6] | BATTERY_BUF[7] << 8) * .07813f);
//    buzzvm_tput(vm);
//    buzzvm_dup(vm);
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "capacity", 1));
//    buzzvm_pushf(vm, BATTERY_BUF[3]);
//    buzzvm_tput(vm);
//    buzzvm_gstore(vm);
//    return vm->state;
// }

// /****************************************/
// /****************************************/

// int buzzkh4_camera_updateblob(buzzvm_t vm, int* blob){
//    //int* blob =get_available_blob();
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "camera_blob", 1));
//    buzzvm_pusht(vm);
//       //push x center
//    buzzvm_dup(vm);
//    buzzvm_pushi(vm, 1);
//    buzzvm_pushi(vm, blob[0]);
//    buzzvm_tput(vm);
//    // push y center of blob
//    buzzvm_dup(vm);
//    buzzvm_pushi(vm, 2);
//    buzzvm_pushi(vm, blob[1]);
//    buzzvm_tput(vm);
//    // push blob size
//    buzzvm_dup(vm);
//    buzzvm_pushi(vm, 3);
//    buzzvm_pushi(vm, blob[2]);
//    buzzvm_tput(vm);
//    // push blob size
//    buzzvm_dup(vm);
//    buzzvm_pushi(vm, 4);
//    buzzvm_pushi(vm, blob[3]);
//    buzzvm_tput(vm);

//    buzzvm_gstore(vm);
//    return vm->state;
// }

// /****************************************/
// /****************************************/

int buzzkh4_abs_position(buzzvm_t vm, float x, float y, float theta) {
  buzzvm_pushs(vm, buzzvm_string_register(vm, "pose", 1));
  buzzvm_pusht(vm);
  buzzobj_t tPoseTable = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);

  //  Create table for i-th read
  buzzvm_pusht(vm);
  buzzobj_t tPosition = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 0));
  buzzvm_pushf(vm, x);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 0));
  buzzvm_pushf(vm, y);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tPoseTable);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "position", 0));
  buzzvm_push(vm, tPosition);
  buzzvm_tput(vm);

  //  Create table for i-th read
  buzzvm_pusht(vm);
  buzzobj_t tOrientation = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tOrientation);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "yaw", 0));
  buzzvm_pushf(vm, theta);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tPoseTable);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "orientation", 0));
  buzzvm_push(vm, tOrientation);
  buzzvm_tput(vm);
  return vm->state;
}

// /****************************************/
// /****************************************/

// int buzzkh4_update_ir(buzzvm_t vm) {
//    static char PROXIMITY_BUF[256];
//    int i;
//    //kh4_proximity_ir(PROXIMITY_BUF, DSPIC);
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity_ir", 1));
//    buzzvm_pusht(vm);
//    for(i = 0; i < 8; i++) {
//       buzzvm_dup(vm);
//       buzzvm_pushi(vm, i+1);
//       buzzvm_pushi(vm, (PROXIMITY_BUF[i*2] | PROXIMITY_BUF[i*2+1] << 8));
//       buzzvm_tput(vm);
//    }
//    buzzvm_gstore(vm);
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "ground_ir", 1));
//    buzzvm_pusht(vm);
//    for(i = 8; i < 12; i++) {
//       buzzvm_dup(vm);
//       buzzvm_pushi(vm, i-7);
//       buzzvm_pushi(vm, (PROXIMITY_BUF[i*2] | PROXIMITY_BUF[i*2+1] << 8));
//       buzzvm_tput(vm);
//    }
//    buzzvm_gstore(vm);
//    return vm->state;
// }

// /****************************************/
// /****************************************/

// int buzzkh4_update_ir_filtered(buzzvm_t vm) {
//    static char PROXIMITY_BUF[256];
//    float ir_gain = expf( - sampling_rate / filter_time_const);
//    int i;
//    // kh4_proximity_ir(PROXIMITY_BUF, DSPIC);
//    buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity", 1));
//    buzzvm_pusht(vm);
//    buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
//    buzzvm_gstore(vm);
//    buzzobj_t tProxRead;

//    for(i = 0; i < 8; i++){
//       /* Fill in the read */
//       int a = (i + 4) % 8;
//       float current_value = (PROXIMITY_BUF[a*2] | PROXIMITY_BUF[a*2+1] << 8) / 1024.0f;
//       float current_value_filtered = ir_table[i] + (1 - ir_gain) * (current_value - ir_table[i]);
//       if(fabs(current_value_filtered - ir_table[i]) < 0.15){
//         ir_table[i] = current_value_filtered;
//       }
//    }

//    for(i = 0; i < 8; i++) {
//       buzzvm_pusht(vm);
//       tProxRead = buzzvm_stack_at(vm, 1);
//       buzzvm_pop(vm);
//       /* Fill in the read */
//       //int a = (i + 4) % 8;
//       //TablePutI(tProxRead, "value", (PROXIMITY_BUF[a*2] | PROXIMITY_BUF[a*2+1] << 8), vm);
//       TablePutF(tProxRead, "value", ir_table[i], vm);
//       int angle = 7 - i;
//       TablePutI(tProxRead, "angle", angle * 45, vm);
//       /* Store read table in the proximity table */
//       TablePutO(tProxTable, i, tProxRead, vm);
//    }

//    return vm->state;
// }

// /****************************************/
// /****************************************/

// int buzzkh4_enable_us(buzzvm_t vm, int value){
//   // in accordance to demo from libkhepera1.1: khepera4_test.c
//   if(value == 1){
//     // kh4_activate_us(31, DSPIC);
//     US_ENABLED = 1;
//   } else {
//     // kh4_activate_us(0, DSPIC);
//     US_ENABLED = 0;
//   }
//   return vm->state;
// }

// int buzzkh4_update_us(buzzvm_t vm){
//   static char PROXIMITY_BUF[256];
//   int i;
//   // kh4_measure_us(PROXIMITY_BUF, DSPIC);
//   buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity_us", 1));
//   buzzvm_pusht(vm);
//   buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
//   buzzvm_gstore(vm);
//   buzzobj_t tProxRead;
//   if(US_ENABLED){
//     for (i = 0; i < 5; i++){
//       buzzvm_pusht(vm);
//       tProxRead = buzzvm_stack_at(vm, 1);
//       buzzvm_pop(vm);
//       TablePutF(tProxRead, "value", (PROXIMITY_BUF[i*2] | PROXIMITY_BUF[i*2+1]<<8), vm);
//       int angle = 90 - (i * 45);
//       if(angle < 0){
//         angle += 360;
//       }
//       TablePutI(tProxRead, "angle", angle, vm);
//       TablePutO(tProxTable, i, tProxRead, vm);
//     }
//   }
//   return vm->state;
// }

/****************************************/
/****************************************/
/*
int buzzkh4_play_sound(buzzvm_t vm, char* filename, int volume){
  char* sound_buff = NULL;
  int data_size;
  short channels;
  short bits_per_sample;
  int sample_rate;
  kb_sound_init();
  switch_speakers_ON_OFF(1);
  mute_speaker(0);
  set_speakers_volume(volume, volume);
  int err;

  fprintf(stderr,"Now playing %s, volume %d!\r\n", filename, volume);

  if ((err=load_wav_file(filename,&sound_buff,&data_size,&channels,&bits_per_sample,&sample_rate))<0)
	{
		fprintf(stderr,"Error: could not open wav file %s, error number %d!\r\n", filename, err);
		free(sound_buff);
		kb_sound_release();
	} else {
    play_buffer(sound_buff,data_size);
    wait_end_of_play();
    free(sound_buff);
  }
  switch_speakers_ON_OFF(0);
  mute_speaker(1);

  return vm->state;
}

*/
/****************************************/
// Buzz table operations
/****************************************/


buzzvm_state TablePutF(buzzobj_t t_table, const char* str_key, float n_value, buzzvm_t m_tBuzzVM) {
   buzzvm_push(m_tBuzzVM, t_table);
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, str_key, 1));
   buzzvm_pushf(m_tBuzzVM, n_value);
   buzzvm_tput(m_tBuzzVM);
   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

buzzvm_state TablePutI(buzzobj_t t_table, const char* str_key, int n_value, buzzvm_t m_tBuzzVM) {
   buzzvm_push(m_tBuzzVM, t_table);
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, str_key, 1));
   buzzvm_pushi(m_tBuzzVM, n_value);
   buzzvm_tput(m_tBuzzVM);
   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

buzzvm_state TablePutO(buzzobj_t t_table, int n_idx, buzzobj_t t_obj, buzzvm_t m_tBuzzVM) {
   buzzvm_push(m_tBuzzVM, t_table);
   buzzvm_pushi(m_tBuzzVM, n_idx);
   buzzvm_push(m_tBuzzVM, t_obj);
   buzzvm_tput(m_tBuzzVM);
   return m_tBuzzVM->state;
}
