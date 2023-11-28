/********************************************************************
* Description:  high-flow-lt
* A HAL component for counting pulses from an Aqua Computer
* High Flow LT flow sensor.
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
MODULE_DESCRIPTION("Measure pulses from a High Flow LT flow sensor to calculate a flow rate.");
MODULE_LICENSE("GPL");

typedef struct {
  hal_bit_t last_signal;
  hal_bit_t *signal;
  hal_float_t *flow_rate;
  hal_float_t *pulses_per_liter;
  hal_float_t *time_window;
  hal_float_t *time;
  hal_u32_t *pulses;
} data_t;

static const char *modname = "high-flow-lt";
static int comp_id;

static int update(void *arg, const hal_funct_args_t *fa) {
  data_t *data = (data_t*)arg;
  long period_ns = fa_period(fa);
  hal_float_t period_s = (hal_float_t)(period_ns)/1000/1000/1000;

  *(data->time) += period_s;

  if(!data->last_signal && *(data->signal)) {
    // signal transitioned from low to high
    *(data->pulses) += 1;
  }

  if(*(data->time) > *(data->time_window)) {
    *(data->flow_rate) = *(data->pulses)/(*(data->time))/(*(data->pulses_per_liter))*60;
    *(data->pulses) = 0;
    *(data->time) = 0;
  }

  data->last_signal = *(data->signal);
};

static int instantiate_instance(const int argc, char* const *argv) {
  data_t *data;
  const char* instname = argv[1];
  int r;

  int inst_id = hal_inst_create(instname, comp_id, sizeof(data_t), (void**)&data);
  if(inst_id < 0) {
    return -1;
  }

  r = hal_pin_bit_newf(HAL_IN, &(data->signal), inst_id, "%s.signal", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.signal'\n", modname, instname);
    return r;
  }

  r = hal_pin_float_newf(HAL_IN, &(data->pulses_per_liter), inst_id, "%s.pulses-per-liter", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.pulses-per-liter'\n", modname, instname);
    return r;
  }

  r = hal_pin_float_newf(HAL_IN, &(data->time_window), inst_id, "%s.time-window", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.time-window'\n", modname, instname);
    return r;
  }

  r = hal_pin_float_newf(HAL_OUT, &(data->flow_rate), inst_id, "%s.flow-rate", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.flow-rate'\n", modname, instname);
    return r;
  }

  r = hal_pin_float_newf(HAL_OUT, &(data->time), inst_id, "%s.time", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.time'\n", modname, instname);
    return r;
  }

  r = hal_pin_u32_newf(HAL_OUT, &(data->pulses), inst_id, "%s.pulses", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.pulses'\n", modname, instname);
    return r;
  }

  *(data->signal) = 0;
  *(data->pulses_per_liter) = 169;
  *(data->time_window) = 1;
  *(data->flow_rate) = 0;
  *(data->time) = 0;
  *(data->pulses) = 0;

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
