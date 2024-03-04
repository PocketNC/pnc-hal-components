/********************************************************************
* Description:  user-message
* An instantiable component for sending a message when transitioning
* an input pin from low to high.
*
* Author: John Allwine <john@pentamachine.com>
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
MODULE_DESCRIPTION("Send a message when input pin transitions from low to high.");
MODULE_LICENSE("GPL");

/*
  RTAPI_MSG_ERR = 1
  RTAPI_MSG_WARN = 2
  RTAPI_MSG_INFO = 3
  RTAPI_MSG_DBG = 4
*/

typedef struct {
  hal_bit_t *in;
  hal_u32_t *type;
  char *message;
  hal_bit_t lastIn;
} data_t;

char* defaultMessage = "This is the default message. Add a message argument using -- to separate it from other parameters: newinst user-message <name> -- <message>";

static const char *modname = "user-message";
static int comp_id;

static int update(void *arg, const hal_funct_args_t *fa) {
  data_t *data = (data_t*)arg;

  if(!data->lastIn && *(data->in)) {
    if(*(data->type) >= 1 && *(data->type) <= 4) {
      msg_level_t t = (msg_level_t)(*(data->type));
      rtapi_print_msg(t, data->message);
    }
  }

  data->lastIn = *(data->in);
};

static int instantiate(const int argc, char* const *argv) {
  data_t *data;
  const char* instname = argv[1];

  rtapi_print_msg(RTAPI_MSG_INFO, "user-message argc %d", argc);
  for(int i = 0; i < argc; i++) {
    rtapi_print_msg(RTAPI_MSG_INFO, "user-message argv %d %s", i, argv[i]);
  }
  int r;

  int inst_id = hal_inst_create(instname, comp_id, sizeof(data_t), (void**)&data);
  if(inst_id < 0) {
    return -1;
  }

  if(argc >= 3) {
    data->message = argv[2];
  } else {
    data->message = defaultMessage;
  }

  r = hal_pin_bit_newf(HAL_IO, &(data->in), inst_id, "%s.in", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.in'\n", modname, instname);
    return r;
  }

  r = hal_pin_u32_newf(HAL_IN, &(data->type), inst_id, "%s.type", instname);
  if(r < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: Error adding pin '%s.type'\n", modname, instname);
    return r;
  }

  *(data->in) = 0;
  *(data->type) = 1;

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
  comp_id = hal_xinit(TYPE_RT, 0, 0, instantiate, NULL, modname);
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
