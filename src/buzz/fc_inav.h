#ifndef FC_INAV
#define FC_INAV


#define CTRL_TIME_PERIOD 0.01
#define MOCAP_TIME_PERIOD 0.07
#define SENS_TIME_PERIOD 0.2
#define DESVEC_TIME_PERIOD 0.5
#define SERIAL_DEVICE "/dev/ttyS0"
#define BAUDRATE 115200
#define PI 3.141592653589

#ifdef __cplusplus
extern "C"
{
#endif

#include <buzz/buzzvm.h>
#include "buzzcognifly_closures.h"
#include <sys/time.h>

extern float CMDS[6];
extern float POSE[4];
extern int DONE;

/*main inav function execution*/
void fc_inav_main();
/*changes the values of command vector*/
int fc_set_RC(buzzvm_t vm);
/*gets voltage value*/
int fc_get_voltage(buzzvm_t vm);
/*lands drone*/
int fc_land(buzzvm_t vm);
/*takes drone off*/
int fc_takeoff(buzzvm_t vm);
/*resets fc*/
int fc_reset(buzzvm_t vm);
/*arms motors*/
int fc_arm(buzzvm_t vm);
/*disarms motors*/
int fc_disarm(buzzvm_t vm);
/*activate mocap functionality*/
int fc_activate_mocap(buzzvm_t vm);
/*deactivate mocap functionality*/
int fc_deactivate_mocap(buzzvm_t vm);
/*tracks input position (in cm)*/
int fc_track_pos(buzzvm_t vm);
/*tracks input velocity (in cm/sec)*/
int fc_track_vel(buzzvm_t vm);
/*activates controlling yaw by changing cmds vector*/
int fc_activate_yaw_control(buzzvm_t vm);
/*deactivates yaw control*/
int fc_deactivate_yaw_control(buzzvm_t vm);
/*sets the thrust value (hence altitude when fc is on cruise mode)*/
int fc_set_thrust(buzzvm_t vm);


/*just a generic waiting function (takes float input)*/
int fc_wait(buzzvm_t vm);
int fc_dummy(buzzvm_t vm);
void WAIT(float);
void rotate_pose();
void vec_to_list();

#ifdef __cplusplus
} // extern "C"
#endif

#endif