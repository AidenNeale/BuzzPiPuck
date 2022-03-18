#include "fc_inav.h"
// #include "include/inc_fc/msp/msp_msg.hpp"
// #include "include/inc_fc/msp/Client.hpp"
#include "/usr/include/eigen3/Eigen/Dense"
#include <iostream>
#include <sys/time.h>
#include <math.h>
// // #include <vector>

// #ifdef __cplusplus
// extern "C"
// {
// #endif

// /*the variables POSE and CMDS are being manipulated either by
// user input functions or otherwise, then CMDS is being used to
// set the RC commands and POSE is being used to send positions
// to the FC to pursue position or velocity*/

float POSE[4] = {10,25,39,40};                  //position: x,y,z and Yaw wrt mocap frame