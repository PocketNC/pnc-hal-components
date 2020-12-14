/********************************************************************
* Description:  torque
*               This file, 'torque.c', is a HAL component that 
*               takes PWM signals from ClearPath motors
*               and outputs torque percentages.
*
* Author: John Allwine <john@pocketnc.com>
* License: GPL Version 2
*    
* Copyright (c) 2020 Pocket NC Company All rights reserved.
*
********************************************************************/

#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "rtapi_errno.h"        /* EINVAL etc */
#include "rtapi_math.h"
#include "hal.h"                /* HAL public API decls */

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

MODULE_AUTHOR("John Allwine");
MODULE_DESCRIPTION("Torque output from PWM feedback from ClearPath motors.");
MODULE_LICENSE("GPL");

typedef struct {
  hal_float_t *duty_cycle;
  hal_float_t *torque;
  hal_float_t *avg_torque;
  hal_float_t *ratio;
  hal_float_t *filter;
  hal_bit_t *fault;
  bool lastFault;
} torque_t;

static torque_t *data;

static int num_axes = 1; // determined by the axes input
static char* axes = "x";
RTAPI_MP_STRING(axes, "Labels for each axis. Each character will represent an axis (i.e. xyz will create 3 input and 3 output pins, an input and output for x, an input and output for y and an input and output for z). Default: x.");


static const char *modname = "torque";
static int comp_id;

static void update(void *arg, long period) {
  for(int i = 0; i < num_axes; i++) {
    const float ratio = *(data[i].ratio);
    const float d = *(data[i].duty_cycle);
    float t = 0;

    if(d >= .05 && d <= .95) {
      if(d < .5) {
        t = 1-((d-.05)/.45);
      } else {
        t = -(d-.5)/.45;
      }
    }

    const float filter = *(data[i].filter); // value between 0 and 1 used to average torque over time
    *(data[i].torque) = ratio*t;
    *(data[i].avg_torque) = *(data[i].avg_torque)*filter + rtapi_fabs(ratio*t)*(1-filter);

    bool fault = d > .99;
    *(data[i].fault) = fault;

    // TODO - move these messages to an E-Stop component that
    // monitors the fault pins and properly logs the reason
    // for an E-Stop.
    if(fault &&  !data[i].lastFault) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: Motor %c fault.", modname, axes[i]);
    }

    if(!fault && data[i].lastFault) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: Cleared fault on motor %c.", modname, axes[i]);
    }

    data[i].lastFault = fault;
  }
}

int rtapi_app_main(void) {
  int retval;
  comp_id = hal_init(modname);
  if(comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
    return -1;
  }

  num_axes = strlen(axes);

  data = hal_malloc(num_axes*sizeof(torque_t));

  for(int i = 0; i  < num_axes; i++) {
    retval = hal_pin_float_newf(HAL_IN, &(data[i].duty_cycle), comp_id, "%s.duty_cycle.%c", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.duty_cycle.%c", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }
    retval = hal_pin_float_newf(HAL_OUT, &(data[i].torque), comp_id, "%s.%c", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.c", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }
    retval = hal_pin_float_newf(HAL_OUT, &(data[i].avg_torque), comp_id, "%s.avg_torque.%c", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.avg_torque.%c", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }
    retval = hal_pin_bit_newf(HAL_OUT, &(data[i].fault), comp_id, "%s.fault.%c", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.fault.%c", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }
    retval = hal_pin_float_newf(HAL_IN, &(data[i].filter), comp_id, "%s.filter.%c", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.filter.%c", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }
    retval = hal_pin_float_newf(HAL_IN, &(data[i].ratio), comp_id, "%s.ratio.%c", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.ratio.%c", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }
    *(data[i].duty_cycle) = 0;
    *(data[i].torque) = 0;
    *(data[i].avg_torque) = 0;
    *(data[i].ratio) = 1;
    *(data[i].filter) = .9;
  }

  char name[20];
  rtapi_snprintf(name, sizeof(name), "%s.funct", modname);
  retval = hal_export_funct(name, update, NULL, 0, 0, comp_id);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: exporting funct failed", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed\n", modname);
  hal_ready(comp_id);
  return 0;
}

void rtapi_app_exit(void) {
  hal_exit(comp_id);
}
