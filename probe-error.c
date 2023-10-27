/********************************************************************
* Description:  probe-error
*               Component that takes in the motion state and error
*               state of the probe in order to report an error.
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
#include "rtapi_math.h"
#include "hal.h"                /* HAL public API decls */

// copied from src/emc/nml_intf/motion_types.h
#define EMC_MOTION_TYPE_PROBING 5

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

MODULE_AUTHOR("John Allwine");
MODULE_DESCRIPTION("Report probe error messages when attempting to probe in an error state.");
MODULE_LICENSE("GPL");

// helper macro for easily creating HAL pins
#define PIN(type,inOrOut,dataName,pinName) \
  retval = hal_pin_##type##_newf((inOrOut), &(data->dataName), comp_id, "%s." #pinName, modname); \
  if(retval < 0) { \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s." #pinName, modname, modname); \
    hal_exit(comp_id); \
    return -1; \
  }


typedef struct {
  hal_s32_t *motion_type;
  hal_bit_t *probe_error;
  hal_bit_t *probe_on;
  hal_bit_t *abort;
} data_t;

static data_t *data;

static const char *modname = "probe-error";
static int comp_id;

static void update(void *arg, long period) {
  hal_bit_t lastAbort = *(data->abort);
  *(data->abort) = *(data->probe_on) && *(data->motion_type) == EMC_MOTION_TYPE_PROBING && *(data->probe_error);

  if(!lastAbort && *(data->abort)) {
    // only send error on transition into abort state
    rtapi_print_msg(RTAPI_MSG_ERR, "Probe is in an error state. Ensure the probe is charged and has line of sight to a receiver.");
  }
}

int rtapi_app_main(void) {
  int retval;
  comp_id = hal_init(modname);
  if(comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
    return -1;
  }

  data = hal_malloc(sizeof(data_t));

  PIN(s32, HAL_IN, motion_type, motion-type);
  PIN(bit, HAL_IN, probe_error, probe-error);
  PIN(bit, HAL_IN, probe_on, probe-on);
  PIN(bit, HAL_OUT, abort, abort);

  *(data->motion_type) = 0;
  *(data->probe_error) = 0;
  *(data->probe_on) = 0;
  *(data->abort) = 0;

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
