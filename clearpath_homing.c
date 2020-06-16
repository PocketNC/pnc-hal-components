/********************************************************************
* Description:  clearpath_homing
*               This file, 'clearpath_homing.c', is a HAL component that 
*               performs the hard stop or specific angle homing routines
*               for Teknic's ClearPath SDSK servo motors.
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

typedef enum {
  UNPOWERED,
  POWERED,
  CYCLE_POWER_OFF,
  CYLCE_POWER_ON,
  BEGIN_HOMING,
  HOMING,
  HOMED
} state_t;

typedef enum {
  HARDSTOP,
  ANGLE
} type_t;

typedef struct {
  // input pins
  hal_bit_t *start;         // trigger the homing process
  hal_float_t *feedback;    // feedback from motor, the duty cycle
  hal_bit_t *home_switch;   // used for specific angle homing (first home to switch, then go to specific angle)
  hal_u32_t *type;          // type of homing sequence (hardstop or continuous)

  // output pins
  hal_bit_t *homed;           // true once axis is homed
  hal_bit_t *homing;          // true while axis is being homed
  hal_bit_t *moving;          // true while jogging into position while homing
  hal_bit_t *speed;           // speed of movement   
  hal_bit_t *enable;          // connect to enable pin for specific axis

  state_t state;
} axis_t;

typedef struct {
  hal_bit_t *machine_on;    // is power on? Must be on in order to enable individual motors
  hal_bit_t *home_all;      // trigger homing sequence for all axes

  axis_t* axis;             // data structure for each axis
} clearpath_t;

static clearpath_t *data;

static int num_axes = 1; // determined by the axes input
static char* axes = "x";
RTAPI_MP_STRING(axes, "Labels for each axis. Each character will represent an axis. Default: x.");

static const char *modname = "clearpath_homing";
static int comp_id;

static void update(void *arg, long period) {
  for(int i = 0; i < num_axes; i++) {
    const type = *(data->axis[i].type);
    switch(type) {
      case HARDSTOP: {
        // Transitions
        
        break;
      }
      case ANGLE:
        break;
      default:
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown axis type", modname);
    }
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

  data = hal_malloc(sizeof(clearpath_t));
  data->axis = hal_malloc(sizeof(axis_t)*num_axes);

  retval = hal_pin_float_newf(HAL_IN, &(data->machine_on), comp_id, "%s.machine_on", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.machine_on", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_float_newf(HAL_IN, &(data->home_all), comp_id, "%s.home_all", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.home_all", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  *(data->machine_on) = 0;
  *(data->home_all) = 0;

  for(int i = 0; i  < num_axes; i++) {
    retval = hal_pin_float_newf(HAL_IN, &(data->axis[i].start), comp_id, "%s.%c.start", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.start", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }
    retval = hal_pin_float_newf(HAL_IN, &(data->axis[i].feedback), comp_id, "%s.%c.feedback", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.feedback", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(data->axis[i].home_switch), comp_id, "%s.%c.home_switch", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.home_switch", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(data->axis[i].type), comp_id, "%s.%c.type", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.type", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(data->axis[i].homed), comp_id, "%s.%c.homed", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.homed", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(data->axis[i].homing), comp_id, "%s.%c.homing", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.homing", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(data->axis[i].moving), comp_id, "%s.%c.moving", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.moving", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(data->axis[i].speed), comp_id, "%s.%c.speed", modname, axes[i]);
    if(retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.%c.speed", modname, modname, axes[i]);
      hal_exit(comp_id);
      return -1;
    }

    *(data->axis[i].start)       = 0;
    *(data->axis[i].feedback)    = 0;
    *(data->axis[i].home_switch) = 0;
    *(data->axis[i].type)        = HARDSTOP;
    *(data->axis[i].homed)       = 0;
    *(data->axis[i].homing)      = 0;
    *(data->axis[i].moving)      = 0;
    *(data->axis[i].speed)       = 0;
    *(data->axis[i].enable)      = 0;
  }

  char name[30];
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
