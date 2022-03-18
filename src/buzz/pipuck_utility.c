#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include "pipuck_utility.h"

/****************************************/
/****************************************/

void pipuck_setup() {
  fprintf(stdout, "Robot setup.\n");
}

/****************************************/
/****************************************/

void pipuck_done() {
  fprintf(stdout, "Robot stopped.\n");
}

/****************************************/
/****************************************/
