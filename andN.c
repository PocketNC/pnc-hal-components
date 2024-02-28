/********************************************************************
* Description:  andN
* A HAL component for performing and logic with
* up to 128 boolean inputs.
*
* Author: John Allwine <john@pocketnc.com>
* License: GPL Version 2
*    
* Copyright (c) 2024 Pocket NC Company All rights reserved.
*
********************************************************************/

#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "rtapi_errno.h"        /* EINVAL etc */
#include "hal.h"                /* HAL public API decls */
#include "hal_priv.h"
#include <sys/mman.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

#define MAX_NUM_INPUTS 128

MODULE_AUTHOR("John Allwine");
MODULE_DESCRIPTION("And up to 128 inputs together.");
MODULE_LICENSE("GPL");

typedef struct {
  hal_bit_t **inputs;
  hal_bit_t *output;
  int numInputs;
} data_t;

static const char *modname = "andN";
static int comp_id;

static int inputs = 2;
RTAPI_IP_INT(inputs, "number of input HAL pins to and together");

static int defaultValue = 1;
RTAPI_IP_INT(defaultValue, "default state of inputs, 0 or 1.");

static int update(void *arg, const hal_funct_args_t *fa) {
  data_t *data = (data_t*)arg;

  int out = 1;
  for(int i = 0; i < data->numInputs; i++) {
    if(!*(data->inputs[i])) {
      out = 0;
      break;
    }
  }

  *(data->output) = out;
};

static int instantiate_instance(const int argc, char* const *argv) {
  data_t *data;
  const char* instname = argv[1];
  int r;

  if(inputs < 2) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error instantiating '%s': inputs must be greater than or equal to 2\n", modname, instname);
    return -1;
  }
  if(inputs > MAX_NUM_INPUTS) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error instantiating '%s': inputs must be less than or equal to %d\n", modname, instname, MAX_NUM_INPUTS);
    return -1;
  }

  int inst_id = hal_inst_create(instname, comp_id, sizeof(data_t), (void**)&data);
  if(inst_id < 0) {
    return -1;
  }

  data->numInputs = inputs;
  data->inputs = hal_malloc(inputs*sizeof(hal_bit_t *));

  for(int i = 0; i < inputs; i++) {
    r = hal_pin_bit_newf(HAL_IN, &(data->inputs[i]), inst_id, "%s.in%d", instname,i);
    *(data->inputs[i]) = (defaultValue != 0);
    if(r < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.in%d'\n", modname, instname, i);
      return r;
    }
  }

  r = hal_pin_bit_newf(HAL_OUT, &(data->output), inst_id, "%s.out", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.out'\n", modname, instname);
    return r;
  }

  hal_export_xfunct_args_t updateArgs = {
    .type = FS_XTHREADFUNC,
    .funct.x = update,
    .arg = data,
    .uses_fp = 1,
    .reentrant = 0,
    .owner_id = inst_id
  };
  r = hal_export_xfunctf(&updateArgs, "%s.funct", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: function export failed: %s\n", modname, instname);
    return r;
  }

  return 0;
}

int rtapi_app_main(void) {
  comp_id = hal_xinit(TYPE_RT, 0, 0, instantiate_instance, NULL, modname);
  if(comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
    return -1;
  }

  hal_ready(comp_id);
  return 0;
}

void rtapi_app_exit(void) {
  hal_exit(comp_id);
}
