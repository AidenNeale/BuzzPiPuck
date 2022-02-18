#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include "pipuck_utility.h"

/****************************************/
/****************************************/

static const int ACC_INC       = 3;
static const int ACC_DIV       = 0;
static const int MIN_SPEED_ACC = 20;
static const int MIN_SPEED_DEC = 1;
static const int MAX_SPEED     = 400; /* mm/sec */

/****************************************/
/****************************************/

void kh4_setup() {
   // Set the libkhepera debug level
}

/****************************************/
/****************************************/

void kh4_done() {
   fprintf(stdout, "Robot stopped.\n");
   CMDS[4] = 1000;
}

/****************************************/
/****************************************/
