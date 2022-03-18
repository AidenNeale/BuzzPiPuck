#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include "pipuck_utility.h"
#include "include/src_puck/PiPuck.h"

/****************************************/
/****************************************/

void pipuck_setup() {
  i2c_initialise();
  fprintf(stdout, "Robot setup.\n");
}

/****************************************/
/****************************************/

void pipuck_done() {
  i2c_destroy();
  fprintf(stdout, "Robot stopped.\n");
}

/****************************************/
/****************************************/
