#define _GNU_SOURCE
#include <stdio.h>

#include "buzz_utility.h"
#include "buzzpipuck_closures.h"
#include <buzz/buzzdebug.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <pthread.h>

/****************************************/
/****************************************/

static buzzvm_t    VM              = 0;
static char*       BO_FNAME        = 0;
static uint8_t*    BO_BUF          = 0;
static buzzdebug_t DBG_INFO        = 0;
static int         MSG_SIZE        = -1;
static int         UDP_LIST_STREAM = -1;
static int         UDP_COMM_STREAM = -1;
static uint8_t*    STREAM_SEND_BUF = NULL;
static int         MSG_RANGE = 1.0;  //Max accepted range for msgs (m)
static int         PACKET_LOSS_FACTOR = 25;
static char        UDP_LIST_STREAM_PORT[10];

// absolute positioning
float abs_x = 0.0, abs_y = 0.0, abs_z = 0.0, abs_theta = 0.0;

static int ROBOT_ID;

/* Pointer to a function that sends a message on the stream */
static void (*STREAM_SEND)() = NULL;

/* PThread handle to manage incoming messages */
static pthread_t INCOMING_MSG_THREAD;

/****************************************/
/****************************************/

/* PThread mutex to manage the list of incoming packets */
static pthread_mutex_t INCOMING_PACKET_MUTEX;

/* List of packets received over the stream */
struct incoming_packet_s {
  /* Id of the message sender */
  int id;
  /* Payload */
  uint16_t* payload;
  /* Next message */
  struct incoming_packet_s* next;
};

/* The list of incoming packets */
static struct incoming_packet_s* PACKETS_FIRST = NULL;
static struct incoming_packet_s* PACKETS_LAST  = NULL;

static struct sockaddr_in server;

void incoming_packet_add(uint16_t id, const uint8_t* pl) {
  // check if the packet is from the current robot
  // if so, extract the absolute position data, and drop the package
  if(id == ROBOT_ID){
    int offset = 0;
    memcpy(&abs_x, pl + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&abs_y, pl + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&abs_z, pl + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&abs_theta, pl + offset, sizeof(float));
    return;
  }

  /* Create packet */
  struct incoming_packet_s* p = (struct incoming_packet_s*)malloc(sizeof(struct incoming_packet_s));
  /* Fill in the data */
  p->id = id;
  p->payload = malloc(MSG_SIZE - sizeof(uint16_t));
  memcpy(p->payload, pl, MSG_SIZE - sizeof(uint16_t));

  p->next = NULL;
  /* Lock mutex */
  pthread_mutex_lock(&INCOMING_PACKET_MUTEX);
  /* Add as last to list */
  if(PACKETS_FIRST != NULL)
    PACKETS_LAST->next = p;
  else
    PACKETS_FIRST = p;
  PACKETS_LAST = p;
  /* Unlock mutex */
  pthread_mutex_unlock(&INCOMING_PACKET_MUTEX);
}

/****************************************/
/****************************************/
/*this function works forever in its own thread listening
for data sent over the UDP Stream*/
void* buzz_stream_incoming_thread_UDP(void* args) {
  /*

  */
   /* Create buffer for message */
   uint8_t* buf = calloc(MSG_SIZE, 1);
   /* Tot bytes left to receive, received up to now, and received at a
    * specific call of recv() */
   ssize_t left, tot, cur;
   while(1) {
      /* Initialize left byte count */
      left = MSG_SIZE;
      tot = 0;
      while(left > 0) {
         cur = recv(UDP_COMM_STREAM, buf + tot, left, 0);
         if(cur < 0) {
            fprintf(stderr, "Error receiving data: %s\n", strerror(errno));
            free(buf);
            return NULL;
         }
         if(cur == 0) {
            fprintf(stderr, "Connection closed by peer\n");
            free(buf);
            DONE = 1;
            return NULL;
         }
         left -= cur;
         tot += cur;
      }
      /* Done receiving data, add packet to list */
      incoming_packet_add(*(uint16_t*)buf, buf + sizeof(uint16_t));
   }
}

void buzz_stream_send_UDP() {
   /* Tot bytes left to send, sent up to now, and sent at a specific
    * call of send() */
   ssize_t left, tot, cur;
   /* Initialize left byte count */
   left = MSG_SIZE;
   tot = 0;
   while(left > 0) {
     cur = sendto(UDP_COMM_STREAM, STREAM_SEND_BUF + tot, left, 0,  (struct sockaddr *)&server, sizeof(server));
      if(cur < 0) {
         fprintf(stderr, "Error receiving data: %s\n", strerror(errno));
         exit(1);
      }
      if(cur == 0) {
         fprintf(stderr, "Connection closed by peer\n");
         exit(1);
      }
      left -= cur;
      tot += cur;
   }
}

/****************************************/
/****************************************/

int buzz_listen_UDP() {

   /* Set up the server name */
   server.sin_family      = AF_INET;                /* Internet Domain    */
   server.sin_port        = htons(4242);            /* Server Port        */
   server.sin_addr.s_addr = inet_addr(SERVER_ADDR); /* Server's Address   */

   sprintf(UDP_LIST_STREAM_PORT, "%d", 24580 + ROBOT_ID) ;

   /* Used to store the return value of the network function calls */
   int retval;
   /* Get information on the available interfaces */
   struct addrinfo hints, *ifaceinfo;
   memset(&hints, 0, sizeof(hints));
   hints.ai_family = AF_INET;       /* Only IPv4 is accepted */
   hints.ai_socktype = SOCK_DGRAM;  /* UDP socket */
   hints.ai_flags = AI_PASSIVE;     /* Necessary for bind() later on */
   retval = getaddrinfo(NULL,
                        UDP_LIST_STREAM_PORT,
                        &hints,
                        &ifaceinfo);
   if(retval != 0) {
      fprintf(stderr, "Error getting local address information: %s\n",
              gai_strerror(retval));
      return 0;
   }
   /* Bind on the first interface available */
   UDP_COMM_STREAM = -1;
   struct addrinfo* iface = NULL;
   for(iface = ifaceinfo;
       (iface != NULL) && (UDP_LIST_STREAM == -1);
       iface = iface->ai_next) {
      UDP_COMM_STREAM = socket(iface->ai_family,
                               iface->ai_socktype,
                               iface->ai_protocol);
      if(UDP_COMM_STREAM > 0) {
         int True = 1;
         if((setsockopt(UDP_COMM_STREAM,
                        SOL_SOCKET,
                        SO_REUSEADDR,
                        &True,
                        sizeof(True)) != -1)
            &&
            (bind(UDP_COMM_STREAM,
                  iface->ai_addr,
                  iface->ai_addrlen) == -1)) {
            close(UDP_LIST_STREAM);
            UDP_COMM_STREAM = -1;
         }
      }
   }
   freeaddrinfo(ifaceinfo);
   if(UDP_COMM_STREAM == -1) {
      fprintf(stderr, "Can't bind socket to any interface\n");
      return 0;
   }
   /* Listen on the socket */
   fprintf(stdout, "Listening on port %s...\n",UDP_LIST_STREAM_PORT);
   /* Ready to communicate through UDP */
   STREAM_SEND = buzz_stream_send_UDP;
   STREAM_SEND_BUF = (uint8_t*)malloc(MSG_SIZE);
   if(pthread_create(&INCOMING_MSG_THREAD, NULL, &buzz_stream_incoming_thread_UDP, NULL) != 0) {
      fprintf(stderr, "Can't create thread: %s\n", strerror(errno));
      close(UDP_COMM_STREAM);
      UDP_COMM_STREAM = -1;
      return 0;
   }
   return 1;
}


int buzz_listen_bt() {
   return 0;
}

int buzz_listen(const char* type, int msg_size, int RID) {
   /* Set the message size */
   MSG_SIZE = msg_size;
   /* Sets the Robot ID */
   ROBOT_ID = RID;
   /* Create the mutex */
   if(pthread_mutex_init(&INCOMING_PACKET_MUTEX, NULL) != 0) {
      fprintf(stderr, "Error initializing the incoming packet mutex: %s\n",
              strerror(errno));
      return 0;
   }
   /* Listen to connections */
   if(strcasecmp(type, "UDP") == 0)
      return buzz_listen_UDP();
   else if(strcasecmp(type, "bt") == 0) //This does nothing useful
      return buzz_listen_bt();
   return 0;
}

/****************************************/
/****************************************/

static const char* buzz_error_info() {
   buzzdebug_entry_t dbg = *buzzdebug_info_get_fromoffset(DBG_INFO, &VM->pc);
   char* msg;
   if(dbg != NULL) {
      asprintf(&msg,
               "%s: execution terminated abnormally at %s:%" PRIu64 ":%" PRIu64 " : %s\n\n",
               BO_FNAME,
               dbg->fname,
               dbg->line,
               dbg->col,
               VM->errormsg);
   }
   else {
      asprintf(&msg,
               "%s: execution terminated abnormally at bytecode offset %d: %s\n\n",
               BO_FNAME,
               VM->pc,
               VM->errormsg);
   }
   return msg;
}

/****************************************/
/****************************************/

static int buzz_register_hooks() {
  buzzvm_pushs(VM,  buzzvm_string_register(VM, "print", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzz_pipuck_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM,  buzzvm_string_register(VM, "log", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzz_pipuck_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM,  buzzvm_string_register(VM, "set_wheels", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, pipuck_set_wheels));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM,  buzzvm_string_register(VM, "set_outer_leds", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, pipuck_set_outer_leds));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM,  buzzvm_string_register(VM, "goto", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, pipuck_goto));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM,  buzzvm_string_register(VM, "sleep", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzz_sleep_ms));
  buzzvm_gstore(VM);
  //  buzzvm_pushs(VM,  buzzvm_string_register(VM, "set_led", 1));
  //  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzz_puck_set_led));
  //  buzzvm_gstore(VM);

   return BUZZVM_STATE_READY;
}

/****************************************/
/****************************************/

int buzz_script_set(const char* bo_filename,
                    const char* bdbg_filename,
		                int robot_id) {
   /* Get hostname */
   char hstnm[30];
   gethostname(hstnm, 30);
   // printf("the hostname is: %s\n",hstnm);
   /* Make numeric id from hostname */
   /* NOTE: here we assume that the hostname is in the format Knn */
   printf("robot id is %d\n",robot_id);
   ROBOT_ID = robot_id;
   /* Reset the Buzz VM */
   if(VM){
     buzzvm_destroy(&VM);
   }

   VM = buzzvm_new(robot_id);
   /* Get rid of debug info */
   if(DBG_INFO) buzzdebug_destroy(&DBG_INFO);
   DBG_INFO = buzzdebug_new();
   /* Read bytecode and fill in data structure */
   FILE* fd = fopen(bo_filename, "rb");
   if(!fd) {
      perror(bo_filename);
      return 0;
   }
   fseek(fd, 0, SEEK_END);
   size_t bcode_size = ftell(fd);
   rewind(fd);
   BO_BUF = (uint8_t*)malloc(bcode_size);
   if(fread(BO_BUF, 1, bcode_size, fd) < bcode_size) {
      perror(bo_filename);
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      fclose(fd);
      return 0;
   }
   fclose(fd);
   /* Read debug information */
   if(!buzzdebug_fromfile(DBG_INFO, bdbg_filename)) {
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      perror(bdbg_filename);
      return 0;
   }
   /* Set byte code */
   if(buzzvm_set_bcode(VM, BO_BUF, bcode_size) != BUZZVM_STATE_READY) {
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      fprintf(stdout, "%s: Error loading Buzz script\n\n", bo_filename);
      return 0;
   }
   /* Register hook functions */
   if(buzz_register_hooks() != BUZZVM_STATE_READY) {
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      fprintf(stdout, "%s: Error registering hooks\n\n", bo_filename);
      return 0;
   }
   /* Save bytecode file name */
   BO_FNAME = strdup(bo_filename);
   /* Execute the global part of the script */
   buzzvm_execute_script(VM);
   /*Set a some constant variables */
   buzzvm_pushs(VM, buzzvm_string_register(VM, "V_TYPE", 1));
   buzzvm_pushi(VM, 0);
   buzzvm_gstore(VM);


   /* Call the Init() function */
   buzzvm_function_call(VM, "init", 0);
   /* Remove useless return value from stack */
   buzzvm_pop(VM);
   /* All OK */
   return 1;
}

/****************************************/
/****************************************/

struct buzzswarm_elem_s {
   buzzdarray_t swarms;
   uint16_t age;
};
typedef struct buzzswarm_elem_s* buzzswarm_elem_t;

void check_swarm_members(const void* key, void* data, void* params) {
   buzzswarm_elem_t e = *(buzzswarm_elem_t*)data;
   int* status = (int*)params;
   if(*status == 3) return;
   if(buzzdarray_size(e->swarms) != 1) {
      fprintf(stderr, "Swarm list size is not 1\n");
      *status = 3;
   }
   else {
      int sid = 1;
      if(*buzzdict_get(VM->swarms, &sid, uint8_t) &&
         buzzdarray_get(e->swarms, 0, uint16_t) != sid) {
         fprintf(stderr, "I am in swarm #%d and neighbor is in %d\n",
                 sid,
                 buzzdarray_get(e->swarms, 0, uint16_t));
         *status = 3;
         return;
      }
      sid = 2;
      if(*buzzdict_get(VM->swarms, &sid, uint8_t) &&
         buzzdarray_get(e->swarms, 0, uint16_t) != sid) {
         fprintf(stderr, "I am in swarm #%d and neighbor is in %d\n",
                 sid,
                 buzzdarray_get(e->swarms, 0, uint16_t));
         *status = 3;
         return;
      }
   }
}

void buzz_script_step() {
  /*
  * Process incoming messages
  */

  /* Reset neighbor information */
  buzzneighbors_reset(VM);
  /* Lock mutex */
  pthread_mutex_lock(&INCOMING_PACKET_MUTEX);
  /* Go through messages and add them to the FIFO */
  struct incoming_packet_s* n;

  while(PACKETS_FIRST) {
    /* Save next packet */
    n = PACKETS_FIRST->next;

    uint8_t* pl = (uint8_t*)PACKETS_FIRST->payload;

    int random_num = rand() % 100; //NOTE: srand initialised in bzzpuck.c (random method could be improved)
    printf("Random Num: %d, Packet Loss Factor: %d", random_num, PACKET_LOSS_FACTOR);
    if (random_num >= PACKET_LOSS_FACTOR) {
      /* Update Buzz neighbors information */
      float distance=0.0, azimuth=0.0, elevation=0.0;
      size_t tot = 0; // Top of Tree
      memcpy(&distance, pl+tot, sizeof(float));
      tot += sizeof(float);
      memcpy(&azimuth, pl+tot, sizeof(float));
      tot += sizeof(float);
      memcpy(&elevation, pl+tot, sizeof(float));
      tot += sizeof(float);

      // Skip the orientation
      tot += sizeof(float);

    //if(x > MSG_RANGE) { // limit the msg range of the nieghbor
      buzzneighbors_add(VM, PACKETS_FIRST->id, distance, azimuth, elevation);
      uint16_t msgsz;
      while(MSG_SIZE - tot > sizeof(uint16_t) && msgsz > 0) {
        /* Get payload size */
        msgsz = *(uint16_t*)(pl + tot); // Get message size
        tot += sizeof(uint16_t);
        /* Make sure the message payload can be read */
        if(msgsz > 0 && msgsz <= MSG_SIZE - tot) {
          /* Append message to the Buzz input message queue */
          buzzinmsg_queue_append(
              VM,
              PACKETS_FIRST->id,
              buzzmsg_payload_frombuffer(pl + tot, msgsz));
          tot += msgsz;
        }
      }
    //}

      /**pretty sure that the loop above can be replaced by:
      buzzinmsg_queue_append(
         VM,
         PACKETS_FIRST->id,
         buzzmsg_payload_frombuffer(pl + tot, (MSG_SIZE - tot)));
      */
    }

    /* Erase packet */
    free(PACKETS_FIRST->payload);
    free(PACKETS_FIRST);
    /* Go to next packet */
    PACKETS_FIRST = n;
  }
  /* The packet list is now empty */
  PACKETS_LAST = NULL;
  /* Unlock mutex */
  pthread_mutex_unlock(&INCOMING_PACKET_MUTEX);
  /* Process messages */
  buzzvm_process_inmsgs(VM);
  /*
  * Update sensors
  */
  buzzkh4_abs_position(VM, abs_x, abs_y, abs_theta);
  POSE[0] = abs_x;
  POSE[1] = abs_y;
  POSE[2] = abs_z; //just a dummy value in cm, to be fixed later by assigning actual z value.
  POSE[3] = abs_theta;

  /*
  * Call Buzz step() function
  */
  if(buzzvm_function_call(VM, "step", 0) != BUZZVM_STATE_READY) {
    fprintf(stderr, "%s: execution terminated abnormally: %s\n\n",
            BO_FNAME,
            buzz_error_info());
    buzzvm_dump(VM);
  }
  /* Remove useless return value from stack */
  buzzvm_pop(VM);
  /*
  * Broadcast messages
  */
  /* Prepare buffer */
  buzzvm_process_outmsgs(VM);

  // TODO: this remains
  memset(STREAM_SEND_BUF, 0, MSG_SIZE);
  *(uint16_t*)STREAM_SEND_BUF = VM->robot;
  ssize_t tot = sizeof(uint16_t);

  /* add local position*/
  memcpy(STREAM_SEND_BUF + tot, &abs_x, sizeof(float));
  tot += sizeof(float);
  memcpy(STREAM_SEND_BUF + tot, &abs_y, sizeof(float));
  tot += sizeof(float);
  memcpy(STREAM_SEND_BUF + tot, &abs_z, sizeof(float));
  tot += sizeof(float);
  memcpy(STREAM_SEND_BUF + tot, &abs_theta, sizeof(float));
  tot += sizeof(float);

  // printf("Robot ID: %d at position: x: %f, y: %f, z: %f, theta: %f\n\r", ROBOT_ID, abs_x, abs_y, abs_z, abs_theta);

  while(1) {
    /* Are there more messages? */
    if(buzzoutmsg_queue_isempty(VM)) break;
    /* Get first message */
    buzzmsg_payload_t m = buzzoutmsg_queue_first(VM);
    /* Make sure it fits the data buffer */
    if(tot + buzzmsg_payload_size(m) + sizeof(uint16_t) > MSG_SIZE) {
        buzzmsg_payload_destroy(&m);
        break;
    }
    /* Add message length to data buffer */
    /* fprintf(stderr, "[DEBUG] send before sz = %u\n", */
    /*         *(uint16_t*)(STREAM_SEND_BUF + 2)); */
    *(uint16_t*)(STREAM_SEND_BUF + tot) = (uint16_t)buzzmsg_payload_size(m);
    tot += sizeof(uint16_t);
    /* fprintf(stderr, "[DEBUG] send after sz = %u\n", */
    /*         *(uint16_t*)(STREAM_SEND_BUF + 2)); */
    /* Add payload to data buffer */
    memcpy(STREAM_SEND_BUF + tot, m->data, buzzmsg_payload_size(m));
    tot += buzzmsg_payload_size(m);
    //fprintf(stderr, "[DEBUG] send before sz = %u\n", *(uint16_t*)(STREAM_SEND_BUF + 2));
    /* Get rid of message */
    buzzoutmsg_queue_next(VM);
    buzzmsg_payload_destroy(&m);
  };

  STREAM_SEND();
  /* Push the swarm size */
  buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS",1));
  buzzvm_pushi(VM, buzzdict_size(VM->swarmmembers)+1);
  buzzvm_gstore(VM);

  /* Print swarm
  buzzswarm_members_print(stdout, VM->swarmmembers, VM->robot);*/
  /* Check swarm state
  int status = 1;
  buzzdict_foreach(VM->swarmmembers, check_swarm_members, &status);
  if(status == 1 &&
    buzzdict_size(VM->swarmmembers) < 9)
    status = 2;
  buzzvm_pushs(VM, buzzvm_string_register(VM, "swarm_status", 1));
  buzzvm_pushi(VM, status);
  buzzvm_gstore(VM);*/
}

/****************************************/
/****************************************/

void buzz_script_destroy() {
   /* Cancel thread */
   pthread_cancel(INCOMING_MSG_THREAD);
   pthread_join(INCOMING_MSG_THREAD, NULL);

   /* Get rid of stream buffer */
   free(STREAM_SEND_BUF);

   /* Get rid of virtual machine */
   if(VM) {
      if(VM->state != BUZZVM_STATE_READY) {
         fprintf(stderr, "%s: execution terminated abnormally: %s\n\n",
                 BO_FNAME,
                 buzz_error_info());
         buzzvm_dump(VM);
      }
      buzzvm_function_call(VM, "destroy", 0);
      buzzvm_destroy(&VM);
      free(BO_FNAME);
      buzzdebug_destroy(&DBG_INFO);
   }
   fprintf(stdout, "Script execution stopped.\n");
}

/****************************************/
/****************************************/

int buzz_script_done() {
   return VM->state != BUZZVM_STATE_READY;
}


