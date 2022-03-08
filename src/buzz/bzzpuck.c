#include <buzz/buzzasm.h>
#include <buzz_utility.h>
#include <pipuck_utility.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
// #include "fc_inav.h"

static int done = 0;
/*
 * Print usage information
 */
void usage(const char* path, int status) {
   fprintf(stderr, "Usage:\n");
   fprintf(stderr, "\t%s <stream> <msg_size> <file.bo> <file.bdb>\n\n", path);
   fprintf(stderr, "== Options ==\n\n");
   fprintf(stderr, "  stream        The stream type: tcp or bt\n");
   fprintf(stderr, "  msg_size      The message size in bytes\n");
   fprintf(stderr, "  file.bo       The Buzz bytecode file\n");
   fprintf(stderr, "  file.bdbg     The Buzz debug file\n");
   fprintf(stderr, "  robotid       The robot id, default 0\n");
   fprintf(stderr, "  server        The commhub server, default localhost\n");
   fprintf(stderr, "  frequency     The Buzz step frequency (default 10Hz)\n\n");
   exit(status);
}

static void ctrlc_handler(int sig) {
   done = 1;
   DONE = 1;
}

int main(int argc, char** argv) {
  /* Parse command line */
  if(argc < 5) usage(argv[0], 0);

  /* The stream type */
  char* stream = argv[1];
  if(strcmp(stream, "tcp") != 0 &&
    strcmp(stream, "bt") != 0) {
    fprintf(stderr, "%s: unknown stream type '%s'\n", argv[0], stream);
    usage(argv[0], 0);
  }

  /* The message size for robot message sending */
  char* endptr;
  int msg_sz = strtol(argv[2], &endptr, 10);

  if(endptr == argv[2] || *endptr != '\0') {
    fprintf(stderr, "%s: can't parse '%s' into a number\n", argv[0], argv[2]);
    return 0;
  }
  if(msg_sz <= 0) {
    fprintf(stderr, "%s: invalid value %d for message size\n", argv[0], msg_sz);
    return 0;
  }

  // ROBOT ID
  int RID = 0;
  if(argc >= 6) {
    RID = strtol(argv[5], &endptr, 10);
  }

  // SERVER: This is the IP Address of the server in which communication occurs
  SERVER_ADDR = "144.32.175.138"; // Default Server Address
  if(argc >= 7) {
    SERVER_ADDR = argv[6];
  }

  // frequency
  FREQUENCY = 100000;
  if(argc == 8) {
    FREQUENCY = 1000000 / strtol(argv[7], &endptr, 10);
  }

  /* The bytecode filename */
  char* bcfname = argv[3];
  /* The debugging information file name */
  char* dbgfname = argv[4];

  /* Wait for connection */
  /* this function invokes the buzz_listen_tcp() in case
  of using tcp stream, which creates a thread that runs the
  function called buzz_stream_incoming_thread_tcp()*/
  if(!buzz_listen(stream, msg_sz)) {
    return 1;
  }

  /* Set CTRL-C handler */
  signal(SIGTERM, ctrlc_handler);
  signal(SIGINT, ctrlc_handler);

  /* Set the Buzz bytecode */
  if(buzz_script_set(bcfname, dbgfname, RID)) {
    static struct timeval t1, t2;
    /* Main loop */
    while(!done && !buzz_script_done()) {
      gettimeofday(&t1, NULL);
      buzz_script_step();
      gettimeofday(&t2, NULL);
      double elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);
      double sleepTime = FREQUENCY - elapsedTime;
      if(sleepTime>0){
        usleep(sleepTime);
      }
    }
    /* Cleanup */
    buzz_script_destroy();
  }
  /* All done */
  return 0;
}
