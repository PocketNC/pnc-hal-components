/********************************************************************
* Description:  reset-pin
* A HAL component for resetting the in pin to the value pin after a
* specific amount of time after detecting a different value on
* the in pin.
*
* Author: John Allwine <john@pocketnc.com>
* License: GPL Version 2
*    
* Copyright (c) 2023 Pocket NC Company All rights reserved.
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

MODULE_AUTHOR("John Allwine");
MODULE_DESCRIPTION("Reset the input pin after a specific amount of time.");
MODULE_LICENSE("GPL");

typedef struct {
  hal_bit_t *in;
  hal_bit_t *out;
  hal_bit_t *value;
  hal_u32_t *time;
  hal_u32_t *delay;
} reset_pin_data_t;

static const char *modname = "reset-pin";
static int comp_id;

static int update(void *arg, const hal_funct_args_t *fa) {
  reset_pin_data_t *data = (reset_pin_data_t*)arg;
  long period_ns = fa_period(fa);
  hal_u32_t period_ms = (hal_u32_t)(period_ns/1000/1000);

  if(*(data->in) == *(data->value)) {
    *(data->time) = 0;
  } else {
    *(data->time) += period_ms;

    if(*(data->time) > *(data->delay)) {
      *(data->in) = *(data->value);
    }
  }

  *(data->out) = *(data->in);
};

static int instantiate_reset_pin(const int argc, char* const *argv) {
  reset_pin_data_t *data;
  const char* instname = argv[1];
  int r;

  int inst_id = hal_inst_create(instname, comp_id, sizeof(reset_pin_data_t), (void**)&data);
  if(inst_id < 0) {
    return -1;
  }

  r = hal_pin_bit_newf(HAL_IO, &(data->in), inst_id, "%s.in", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.in'\n", modname, instname);
    return r;
  }

  r = hal_pin_bit_newf(HAL_IN, &(data->value), inst_id, "%s.value", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.value'\n", modname, instname);
    return r;
  }

  r = hal_pin_u32_newf(HAL_IN, &(data->delay), inst_id, "%s.delay", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.delay'\n", modname, instname);
    return r;
  }

  r = hal_pin_bit_newf(HAL_OUT, &(data->out), inst_id, "%s.out", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.out'\n", modname, instname);
    return r;
  }

  r = hal_pin_u32_newf(HAL_OUT, &(data->time), inst_id, "%s.time", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.time'\n", modname, instname);
    return r;
  }

  *(data->in) = 0;
  *(data->out) = 0;
  *(data->value) = 0;
  *(data->time) = 0;
  *(data->delay) = 100;

  hal_export_xfunct_args_t updateArgs = {
    .type = FS_XTHREADFUNC,
    .funct.x = update,
    .arg = data,
    .uses_fp = 0,
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
  comp_id = hal_xinit(TYPE_RT, 0, 0, instantiate_reset_pin, NULL, modname);
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
