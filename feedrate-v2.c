/********************************************************************
* Description:  feedrate-v2
*               This file, 'feedrate-v2.c', is a HAL component that 
*               takes X, Y, Z, A and B velocities and outputs
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

// helper macro for easily creating HAL pins
#define PIN(type,inOrOut,dataName,pinName) \
  retval = hal_pin_##type##_newf((inOrOut), &(data->dataName), comp_id, "%s." #pinName, modname); \
  if(retval < 0) { \
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: could not create pin %s." #pinName, modname, modname); \
    hal_exit(comp_id); \
    return -1; \
  }


typedef struct {
  float lastX;
  float lastY;
  float lastZ;
  float lastA;
  float lastB;
  hal_float_t *x;
  hal_float_t *y;
  hal_float_t *z;
  hal_float_t *tz;
  hal_float_t *a;
  hal_float_t *b;
  hal_float_t *feedrate;

  hal_float_t *xv;
  hal_float_t *yv;
  hal_float_t *zv;
  hal_float_t *av;
  hal_float_t *bv;
} data_t;

static data_t *data;

static const char *modname = "feedrate-v2";
static int comp_id;

static void update(void *arg, long period) {
  const float X = *(data->x);
  const float Y = *(data->y);
  const float Z = *(data->z)-*(data->tz);
  const float A = (*(data->a))*PI/180;
  const float B = (*(data->b))*PI/180;

  const float dt = .001;
  const float xv = (X-data->lastX)/dt;
  const float yv = (Y-data->lastY)/dt;
  const float zv = (Z-data->lastZ)/dt;
  const float av = (A-data->lastA)/dt;
  const float bv = (B-data->lastB)/dt;

  const float CA = rtapi_cos(-A);
  const float SA = rtapi_sin(-A);
  const float CB = rtapi_cos(-B);
  const float SB = rtapi_sin(-B);

  const float omegaX = av;
  const float omegaY = -SA*bv;
  const float omegaZ =  CA*bv;

  // linear velocity
  const float lx = xv;
  const float ly = yv;
  const float lz = zv;

  // rotational velocity, r cross omega

  const float rx = Y*omegaZ-Z*omegaY;
  const float ry = Z*omegaX-X*omegaZ;
  const float rz = X*omegaY-Y*omegaX;

  const float vx = lx+rx;
  const float vy = ly+ry;
  const float vz = lz+rz;

  const float feedrate = rtapi_sqrt((vx*vx)+(vy*vy)+(vz*vz)); 
  *(data->feedrate) = feedrate;
  *(data->xv) = xv;
  *(data->yv) = yv;
  *(data->zv) = zv;
  *(data->av) = av*180/PI;
  *(data->bv) = bv*180/PI;

  data->lastX = X;
  data->lastY = Y;
  data->lastZ = Z;
  data->lastA = A;
  data->lastB = B;
}

int rtapi_app_main(void) {
  int retval;
  comp_id = hal_init(modname);
  if(comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
    return -1;
  }

  data = hal_malloc(sizeof(data_t));

  PIN(float, HAL_IN, x, x);
  PIN(float, HAL_IN, y, y);
  PIN(float, HAL_IN, z, z);
  PIN(float, HAL_IN, tz, tz);
  PIN(float, HAL_IN, a, a);
  PIN(float, HAL_IN, b, b);
  PIN(float, HAL_OUT, feedrate, feedrate);
  PIN(float, HAL_OUT, xv, xv);
  PIN(float, HAL_OUT, yv, yv);
  PIN(float, HAL_OUT, zv, zv);
  PIN(float, HAL_OUT, av, av);
  PIN(float, HAL_OUT, bv, bv);

  *(data->x) = 0;
  *(data->y) = 0;
  *(data->z) = 0;
  *(data->a) = 0;
  *(data->b) = 0;
  *(data->feedrate) = 0;
  *(data->xv) = 0;
  *(data->yv) = 0;
  *(data->zv) = 0;
  *(data->av) = 0;
  *(data->bv) = 0;

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
