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
// static pthread_mutex_t LOCK_CMDS;

// /*this function keeps sending command messages and
// collects needed information all the time, or else fc
// goes to failsafe mode and freezes*/
// void* fc_main_thread(void* args)
// {
//     std::vector<uint16_t> cmds(6, 1500);
//     std::vector<int16_t> mocap_data(5, 0);
//     std::vector<uint16_t> desired_vec(7, 1);

//     // std::vector<uint16_t> cmds(6, 1500);
//     msp::client::Client client;
//     client.setLoggingLevel(msp::client::LoggingLevel::SILENT);
//     client.setVariant(msp::FirmwareVariant::INAV);
//     client.Start(SERIAL_DEVICE, BAUDRATE);
//     msp::FirmwareVariant fw_variant = msp::FirmwareVariant::INAV;

//     // rebooting the drone
//     bool reb = false;
//     msp::ByteVector  reboot_data = msp::ByteVector(0);
//     reb = client.sendData(msp::ID::MSP_REBOOT,reboot_data);
//     client.Stop();
//     if(reb) printf("reboot successfull\n");
//     WAIT(15.0);
//     client.start(SERIAL_DEVICE, BAUDRATE);

//     /*setting up messages we need*/
//     msp::msg::Debug debug(fw_variant);    //for debug messages in case we need them
//     msp::msg::SetMocap mocap(fw_variant); //for sending the mocap data to fc
//     // msp::msg::Analog analog(fw_variant);  //for voltage reading
//     msp::msg::SetRawRc rc(fw_variant);    //for sending commands to fc
//     msp::msg::SetDesVec desired_vector(fw_variant); //for sending desired position or velocity
//     msp::msg::CogniflyAnalog cognifly_analog(fw_variant);
//     // msp::msg::InavAnalog inav_analog(fw_variant);

//     cmds[2] = 900;      //low thrust
//     cmds[4] = 1000;     //motors not armed initially
//     cmds[5] = 1800;     //NAV_POSHOLD mode initially

//     /*setting up some time variables that we need*/
//     struct timeval begin, current_time, last_ctrl_tmr, last_sens_tmr, last_mocap_tmr,last_desVec_tmr;
//     gettimeofday(&begin, 0);
//     gettimeofday(&last_ctrl_tmr,0);
//     gettimeofday(&last_sens_tmr,0);
//     gettimeofday(&last_mocap_tmr,0);
//     gettimeofday(&last_desVec_tmr,0);

//     long sec,usec;

//     /*this loop goes on forever in an independent thread*/
//     while(DONE != 1){
//         int i;
//         rotate_pose();
//         vec_to_list();
//         // setting appropriate messages contents
//         for(i = 0; i < 6; i++)cmds[i] = CMDS[i];
//         for(i = 0; i < 7; i++)desired_vec[i] = DESIRED_LIST[i];
//         for(i = 1; i < 5; i++){
//             if(i == 4){
//                 mocap_data[i] = -POSE[i-1]*180/PI * 10;
//             }
//             else{
//                 // mocap_data[i] = POSE[i-1]*100;
//                 mocap_data[i] = POS_ROT[i-1]*100;
//             }
//         }

//         //check if we have received an order to reset
//         if(reset_flag)
//         {
//             // reb = client.sendData(msp::ID::MSP_REBOOT,reboot_data);
//             // client.Stop();
//             // if(reb) std::cout<<"reboot successful and waiting\r\n";
//             // WAIT(10);
//             // client.start(SERIAL_DEVICE, BAUDRATE);
//             reset_flag = false;
//         }

//         gettimeofday(&current_time, 0);
//         long DT = current_time.tv_sec - begin.tv_sec;
//         // printf("her we go...\n");
//         // if(DT > 15 && DT < 20)cmds[4] = 1800;
//         // else cmds[4] = 1000;

//         //apply yaw control in case it is activated
//         if(control_yaw_active){
//             float kp_yaw = 1;
//             float error_yaw = desired_yaw - POSE[3]*180/PI;
//             int yaw_threshold = 4;
//             if(error_yaw > yaw_threshold || error_yaw < -yaw_threshold){
//                 float control_action_yaw = kp_yaw * error_yaw;
//                 if(control_action_yaw > 500) control_action_yaw = 500;
//                 else if(control_action_yaw < -500) control_action_yaw = -500;
//                 cmds[3] = 1500 - kp_yaw * control_action_yaw;
//             }
//         }

//         //check on the validity of the mocap ... only check when mocap is activated in the step function
//         if(mocap_active){
//             float delta_pose_x = POSE[0] - old_pose[0];
//             float delta_pose_y = POSE[1] - old_pose[1];
//             float eps = 0.0000001;
//             //if the difference between old and new readings is very small, then readings are being repetitve
//             if(delta_pose_x == 0  && delta_pose_y == 0 ){
//                 invalid_mocap_couter++;
//             }
//             else{
//                 invalid_mocap_couter = 0;
//             }
//             //if we have more than a certain number of repetitions
//             if(invalid_mocap_couter > 10000000){
//                 invalid_mocap_flag = true;
//                 printf("The mocap readings recieved are currently invalid!!\t %d\n",invalid_mocap_couter);
//             }
//             else invalid_mocap_flag = false;

//             old_pose[0] = POSE[0];
//             old_pose[1] = POSE[1];
//         }

//         /* Sending RC control messages*/
//         sec = current_time.tv_sec - last_ctrl_tmr.tv_sec;
//         usec = current_time.tv_usec - last_ctrl_tmr.tv_usec; //this time is micro seconds
//         double ctrl_time_elapsed = sec + usec*1e-6;
//         if(ctrl_time_elapsed > CTRL_TIME_PERIOD){
//             // std::cout<<DT<<std::endl;
//             gettimeofday(&last_ctrl_tmr,0);
//             rc.channels = cmds;
//             reb = client.sendData(rc.id(),rc.encode());
//             if(reb){}
//         }

//         /* Sending mocap value messages*/
//         sec = current_time.tv_sec - last_mocap_tmr.tv_sec;
//         usec = current_time.tv_usec - last_mocap_tmr.tv_usec; //this time is micro seconds
//         double mocap_time_elapsed = sec + usec*1e-6;
//         if(mocap_time_elapsed > MOCAP_TIME_PERIOD && mocap_active){
//             gettimeofday(&last_mocap_tmr,0);
//             //increment the counter that activates mocap in case the readings are not repetitive
//             //this should fall back to onboard sensors if mocap is invalid
//             if(!invalid_mocap_flag) mocap_data[0] = mocap_data[0] + 1;
//             mocap.pose = mocap_data;
//             reb = client.sendData(mocap.id(),mocap.encode());
//         }

//         /* Sending desired vector messages*/
//         sec = current_time.tv_sec - last_desVec_tmr.tv_sec;
//         usec = current_time.tv_usec - last_desVec_tmr.tv_usec; //this time is micro seconds
//         double desVec_time_elapsed = sec + usec*1e-6;
//         if(desVec_time_elapsed > DESVEC_TIME_PERIOD){ //will go through if desired_vector_mode != 0
//             gettimeofday(&last_desVec_tmr,0);
//             if(invalid_mocap_flag){
//                 desired_vec[0] = 0; //if mocap is invalid, we don't want to follow speed or position
//                 desired_vec[1] = 0;
//                 desired_vec[2] = 0;
//             }
//             desired_vector.vec = desired_vec;
//             reb = client.sendData(desired_vector.id(),desired_vector.encode());

//             // printf("desired list:  \n");
//             // for (std::vector<uint16_t>::const_iterator j = desired_vec.begin(); j != desired_vec.end(); ++j)
//             // std::cout << *j << ' ';
//             // printf("\n");
//             // for (i = 0; i < 7; i++){
//             //     printf("%d\t",DESIRED_LIST[i]);
//             // }
//             // printf("\n");
//         }

//         /* Collecting data and debug messages*/
//         sec = current_time.tv_sec - last_sens_tmr.tv_sec;
//         usec = current_time.tv_usec - last_sens_tmr.tv_usec; //this time is micro seconds
//         double sens_time_elapsed = sec + usec*1e-6;
//         if(sens_time_elapsed > SENS_TIME_PERIOD){
//             gettimeofday(&last_sens_tmr,0);
//             std::cout<<DT<<std::endl;
//             // printf("POSE X:%0.2f\tY:%0.2f\tTheta:%0.2f\n",POSE[0],POSE[1],POSE[3]);
//             // if(client.sendMessage(analog) == 1){
//             //     fc_voltage = analog.vbat;
//             //     // std::cout << "time: " << DT << "  voltage is  "<<analog.vbat<<std::endl;
//             // }

//             if(client.sendMessage(cognifly_analog) == 1){
//                 fc_voltage = cognifly_analog.vbat;
//                 // std::cout << "time: " << DT << "  voltage is  "<<cognifly_analog.vbat<<std::endl;
//             }

//             if(client.sendMessage(debug) == 1) {
//                 // std::cout<<"mocap data to be sent "<<mocap_data[1]<<"  "<<mocap_data[2]<<"  "<<mocap_data[3]<<"   "<<mocap_data[4]<<std::endl;
//                 std::cout << "#Debug message:" << std::endl;
//                 std::cout << debug.debug1 << " , " <<
//                              debug.debug2 << " , " <<
//                              debug.debug3 << " , " <<
//                              debug.debug4 << std::endl;
//             }
//             else printf("no debug!");
//         }
//     }
//     if(DONE == 1){
//         // landing the drone
//         for(int ii = 0; ii < 50; ii++){
//             cmds[2] = 1000;
//             rc.channels = cmds;
//             client.sendData(rc.id(),rc.encode());
//             WAIT(0.01);
//         }

//         // disarming the drone
//         for(int ii = 0; ii < 100; ii++){
//             cmds[4] = 1000;
//             rc.channels = cmds;
//             client.sendData(rc.id(),rc.encode());
//             WAIT(0.01);
//         }
//     }
//     printf("done with the while loop\n");
//     // pthread_mutex_destroy(&LOCK_CMDS);
// }

// void fc_inav_main()
// {
//     // if(pthread_mutex_init(&LOCK_CMDS, NULL) != 0) {
//     //   fprintf(stderr, "Error initializing the command lock mutex: %s\n",
//     //           strerror(errno));
//     //   return ;
//     // }

//     if(pthread_create(&FC_THREAD, NULL, &fc_main_thread, NULL) != 0) {
//       fprintf(stderr, "Can't create FC_INAV thread: %s\n", strerror(errno));
//    }
// }

// int fc_set_RC(buzzvm_t vm)
// {
//     buzzvm_lnum_assert(vm, 6);
//     /* Push the vector components */
//     buzzvm_lload(vm, 1);
//     buzzvm_lload(vm, 2);
//     buzzvm_lload(vm, 3);
//     buzzvm_lload(vm, 4);
//     buzzvm_lload(vm, 5);
//     buzzvm_lload(vm, 6);
//     buzzvm_type_assert(vm, 6, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 5, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);

//     // pthread_mutex_lock(&LOCK_CMDS);
//     CMDS[0]=buzzvm_stack_at(vm, 6)->f.value; //roll
//     CMDS[1]=buzzvm_stack_at(vm, 5)->f.value; //pitch
//     CMDS[2]=buzzvm_stack_at(vm, 4)->f.value; //throttle
//     CMDS[3]=buzzvm_stack_at(vm, 3)->f.value; //yaw
//     CMDS[4]=buzzvm_stack_at(vm, 2)->f.value; //aux1
//     CMDS[5]=buzzvm_stack_at(vm, 1)->f.value; //aux2
//     // pthread_mutex_unlock(&LOCK_CMDS);

//     return buzzvm_ret0(vm);
// }

// int fc_get_voltage(buzzvm_t vm)
// {
//     buzzvm_pushs(vm, buzzvm_string_register(vm, "fc_voltage", 1));
//     buzzvm_pushf(vm,fc_voltage);
//     buzzvm_gstore(vm);
//     return buzzvm_ret0(vm);
// }

// int fc_land(buzzvm_t vm)
// {
//     CMDS[2] = 1000;
//     return buzzvm_ret0(vm);
// }

// int fc_takeoff(buzzvm_t vm)
// {
//     CMDS[2] = takeoff_thrust;
//     return buzzvm_ret0(vm);
// }

// int fc_reset(buzzvm_t vm)
// {
//     reset_flag = true;
//     return buzzvm_ret0(vm);
// }

// int fc_arm(buzzvm_t vm)
// {
//     CMDS[4] = 1800;
//     return buzzvm_ret0(vm);
// }

// int fc_disarm(buzzvm_t vm)
// {
//     CMDS[4] = 1000;
//     return buzzvm_ret0(vm);
// }

// int fc_activate_mocap(buzzvm_t vm)
// {
//     mocap_active = true;
//     return buzzvm_ret0(vm);
// }

// int fc_activate_yaw_control(buzzvm_t vm)
// {
//     control_yaw_active = true;
//     return buzzvm_ret0(vm);
// }

// int fc_deactivate_mocap(buzzvm_t vm)
// {
//     mocap_active = false;
//     return buzzvm_ret0(vm);
// }

// int fc_deactivate_yaw_control(buzzvm_t vm)
// {
//     control_yaw_active = false;
//     return buzzvm_ret0(vm);
// }

// int fc_set_thrust(buzzvm_t vm)
// {
//     buzzvm_lnum_assert(vm, 1);
//     buzzvm_lload(vm, 1);
//     buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
//     takeoff_thrust = buzzvm_stack_at(vm, 1)->i.value;
//     return buzzvm_ret0(vm);
// }

// int fc_track_pos(buzzvm_t vm)
// {
//     desired_vector_mode = 1;
//     buzzvm_lnum_assert(vm, 3);
//     /* Push the vector components */
//     buzzvm_lload(vm, 1);
//     buzzvm_lload(vm, 2);
//     buzzvm_lload(vm, 3);
//     buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);

//     // pthread_mutex_lock(&LOCK_CMDS);
//     DESIRED[0]=buzzvm_stack_at(vm, 3)->f.value; //x_desired
//     DESIRED[1]=buzzvm_stack_at(vm, 2)->f.value; //y_desired
//     DESIRED[2]=buzzvm_stack_at(vm, 1)->f.value; //z_desired

//     return buzzvm_ret0(vm);
// }

// int fc_track_vel(buzzvm_t vm)
// {
//     desired_vector_mode = 2;
//     buzzvm_lnum_assert(vm, 3);
//     /* Push the vector components */
//     buzzvm_lload(vm, 1);
//     buzzvm_lload(vm, 2);
//     buzzvm_lload(vm, 3);
//     buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
//     buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);

//     // pthread_mutex_lock(&LOCK_CMDS);
//     DESIRED[0]=buzzvm_stack_at(vm, 3)->f.value; //x_desired
//     DESIRED[1]=buzzvm_stack_at(vm, 2)->f.value; //y_desired
//     DESIRED[2]=buzzvm_stack_at(vm, 1)->f.value; //z_desired

//     return buzzvm_ret0(vm);
// }



// int fc_wait(buzzvm_t vm)
// {
//     printf("entered the wait function...\n");
//     buzzvm_lnum_assert(vm, 1);
//     /* Push the vector components */
//     buzzvm_lload(vm, 1);
//     buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
//     float wait_time = buzzvm_stack_at(vm, 1)->f.value;
//     struct timeval start_waiting,current_waiting;
//     gettimeofday(&start_waiting, 0);
//     double dt_wait = 0;
//     long sec_,usec_;

//     while(dt_wait < wait_time)
//     {
//         // printf("waiting...\n");
//         gettimeofday(&current_waiting, 0);
//         sec_ = current_waiting.tv_sec - start_waiting.tv_sec;
//         usec_ = current_waiting.tv_usec - start_waiting.tv_usec; //this time is micro seconds
//         dt_wait = sec_ + usec_*1e-6;
//     }

//     return buzzvm_ret0(vm);
// }

// void WAIT(float a)
// {
//     printf("waiting in the waiting function");
//     struct timeval start_waiting,current_waiting;
//     gettimeofday(&start_waiting, 0);
//     double dt_wait = 0;
//     long sec_,usec_;

//     while(dt_wait < a)
//     {
//         // printf("waiting...\n");
//         gettimeofday(&current_waiting, 0);
//         sec_ = current_waiting.tv_sec - start_waiting.tv_sec;
//         usec_ = current_waiting.tv_usec - start_waiting.tv_usec; //this time is micro seconds
//         dt_wait = sec_ + usec_*1e-6;
//     }
// }

// // just a note for future reference: if you forget to put
// //"return buzzvm_ret0(vm)", the function will act weirdly if
// //it is being called (like a funtion calls itself more than
// // once when it was supposed to be called once, stuff like that)


// void rotate_pose()
// {
//     float phi = PI;
//     float theta = 0;
//     float psi = -POSE[3]; //which is the negative of the yaw angle in radians
//     Eigen::Matrix3d Rx;
//     Eigen::Matrix3d Ry;
//     Eigen::Matrix3d Rz;

//     Rx <<  1,               0,              0,
//            0,               cos(phi),     -sin(phi),
//            0,               sin(phi),     cos(phi);

//     Ry <<  cos(theta),        0,             sin(theta),
//            0,                 1,               0,
//            -sin(theta),       0,             cos(theta);

//     Rz <<  cos(psi),     -sin(psi),     0,
//            sin(psi),     cos(psi),      0,
//            0,               0,          1;

//     Eigen::Vector3d v(POSE[0],POSE[1],POSE[2]);
//     Eigen::Matrix3d R = Rx * Ry * Rz;
//     Eigen::Vector3d V = R * v;

//     POS_ROT[0] = V(0);
//     POS_ROT[1] = V(1);
//     POS_ROT[2] = V(2);

//     // POS_ROT[0] = POSE[0];
//     // POS_ROT[1] = -POSE[1];
//     // POS_ROT[2] = POSE[2];
// }

// void vec_to_list(){ //mode 0: poshold, 1: pos track, 2: vel track
//     DESIRED_LIST[0] = desired_vector_mode;

//     float phi = PI;
//     float theta = 0;
//     float psi = POSE[3]-10*PI/180.0;

//     Eigen::Matrix3d Rx;
//     Eigen::Matrix3d Ry;
//     Eigen::Matrix3d Rz;

//     Rx <<  1,               0,              0,
//            0,               cos(phi),     -sin(phi),
//            0,               sin(phi),     cos(phi);

//     Ry <<  cos(theta),        0,             sin(theta),
//            0,                 1,               0,
//            -sin(theta),       0,             cos(theta);

//     Rz <<  cos(psi),     -sin(psi),     0,
//            sin(psi),     cos(psi),      0,
//            0,               0,          1;

//     Eigen::Vector3d v(DESIRED[0],DESIRED[1],DESIRED[2]);
//     Eigen::Matrix3d R = Rx * Ry * Rz;
//     Eigen::Vector3d V = R * v;

//     int i;
//     // for(i = 0; i < 3; i++){
//     //     DESIRED_LIST[i+1] = abs(DESIRED[i]);
//     //     if(DESIRED[i] < 0){
//     //         DESIRED_LIST[1+3+i] = 0;
//     //     }
//     // }
//     for(i = 0; i < 3; i++){
//         DESIRED_LIST[i+1] = abs(V[i]);
//         if(V[i] < 0){
//             DESIRED_LIST[1+3+i] = 0;
//         }
//         else{
//             DESIRED_LIST[1+3+i] = 1;
//         }
//     }
// }

// int fc_dummy(buzzvm_t vm)
// {
//     printf("this is a dummy function!!!\n");
//     return buzzvm_ret0(vm);
// }
// #ifdef __cplusplus
// } // extern "C"
// #endif