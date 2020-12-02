/********************************************************************
* Description:  feedrate
*               This file, 'feedrate.c', is a HAL component that 
*               takes X, Y, Z, B and C velocities and outputs
*               a single feed rate that represents the speed
*               of the tool tip relative to the work piece.
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
MODULE_DESCRIPTION("Feed rate calculation relative to work piece.");
MODULE_LICENSE("GPL");

#define PI 3.141592653589739

typedef struct {
  float lastX;
  float lastY;
  float lastZ;
  float lastB;
  float lastC;
  hal_float_t *x;
  hal_float_t *y;
  hal_float_t *z;
  hal_float_t *tz;
  hal_float_t *b;
  hal_float_t *c;
  hal_float_t *feedrate;
} data_t;

static data_t *data;

static const char *modname = "feedrate";
static int comp_id;

static void update(void *arg, long period) {
  const float X = *(data->x);
  const float Y = *(data->y);
  const float Z = *(data->z)-*(data->tz);
  const float B = (*(data->b))*PI/180;
  const float C = (*(data->c))*PI/180;

  const float dt = .001;
  const float xv = (X-data->lastX)/dt;
  const float yv = (Y-data->lastY)/dt;
  const float zv = (Z-data->lastZ)/dt;
  const float bv = (B-data->lastB)/dt;
  const float cv = (C-data->lastC)/dt;

  const float CB = rtapi_cos(B);
  const float SB = rtapi_sin(B);
  const float CC = rtapi_cos(C);
  const float SC = rtapi_sin(C);

  const float Qx = CC*CB*X-SC*CB*Y+SB*Z;
  const float Qy = SC*X+CC*Y;
  const float Qz = -CC*SB*X+SC*SB*Y+CB*Z;

  const float lx = CC*CB*xv-SC*CB*yv+SB*zv;
  const float ly = SC*xv+CC*yv;
  const float lz = -CC*SB*xv+SC*SB*yv+CB*zv;

  const float rx =  -bv*Qz + cv*Qy;
  const float ry =        - cv*Qx;
  const float rz = bv*Qx;

  const float vx = lx+rx;
  const float vy = ly+ry;
  const float vz = lz+rz;

  *(data->feedrate) = rtapi_sqrt((vx*vx)+(vy*vy)+(vz*vz)); 

  data->lastX = X;
  data->lastY = Y;
  data->lastZ = Z;
  data->lastB = B;
  data->lastC = C;
}

int rtapi_app_main(void) {
  int retval;
  comp_id = hal_init(modname);
  if(comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
    return -1;
  }

  data = hal_malloc(sizeof(data_t));

  retval = hal_pin_float_newf(HAL_IN, &(data->x), comp_id, "%s.x", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.x", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_float_newf(HAL_IN, &(data->y), comp_id, "%s.y", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.y", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_float_newf(HAL_IN, &(data->z), comp_id, "%s.z", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.z", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_float_newf(HAL_IN, &(data->tz), comp_id, "%s.tz", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.tz", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_float_newf(HAL_IN, &(data->b), comp_id, "%s.b", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.b", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_float_newf(HAL_IN, &(data->c), comp_id, "%s.c", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.c", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_float_newf(HAL_OUT, &(data->feedrate), comp_id, "%s.feedrate", modname);
  if(retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s.feedrate", modname, modname);
    hal_exit(comp_id);
    return -1;
  }

  *(data->x) = 0;
  *(data->y) = 0;
  *(data->z) = 0;
  *(data->b) = 0;
  *(data->c) = 0;
  *(data->feedrate) = 0;

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
