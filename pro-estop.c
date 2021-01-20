/********************************************************************
* Description:  pro-estop
*               This file, 'pro-estop.c', is a HAL component that 
*               has inputs for various E-Stop conditions and will
*               output signals to iocontrol and halui to control
*               E-Stop state.
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
MODULE_DESCRIPTION("E-Stop conditions on Pocket NC pro machine.");
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
  // Latched button state variables.
  // We capture when the physical E-Stop button
  // was pushed or released for timing or for
  // making assumptions about motor/spindle faults
  // (the motors and VFD will report faults when
  // they aren't powered, which they own't be if
  // the physical E-Stop button is pressed).
  hal_bit_t buttonPushed;
  hal_bit_t buttonReleased;

  // Latched fault variables.
  // When a fault is detected these variables will stay high
  // until reset by the user by clicking the E-Stop button in
  // the UI.
  hal_bit_t xFaulted;
  hal_bit_t yFaulted;
  hal_bit_t zFaulted;
  hal_bit_t bFaulted;
  hal_bit_t cFaulted;

  hal_bit_t xFErrored;
  hal_bit_t yFErrored;
  hal_bit_t zFErrored;
  hal_bit_t bFErrored;
  hal_bit_t cFErrored;

  hal_bit_t estop;
  hal_bit_t estopped;

  hal_s32_t spindleErroredWithCode;
  hal_bit_t spindleModbusNotOk;

  // Fault variables.
  // These variables show the current state of various 
  // conditions that trigger a fault.
  hal_bit_t *xFault;           // X fault as reported from the ClearPath motors. Connect to torque.fault.x.
  hal_bit_t *yFault;           // Y fault as reported from the ClearPath motors. Connect to torque.fault.y.
  hal_bit_t *zFault;           // Z fault as reported from the ClearPath motors. Connect to torque.fault.z.
  hal_bit_t *bFault;           // B fault as reported from the ClearPath motors. Connect to torque.fault.b.
  hal_bit_t *cFault;           // C fault as reported from the ClearPath motors. Connect to torque.fault.c.
  hal_bit_t *button;           // Reports the state of the physical E-Stop button that disconnects power to the motors via a relay.
  hal_s32_t *spindleErrorCode; // Reports an error code from the VFD that controls the spindle. Connect to spindle-vfd.error-code.
  hal_bit_t *spindleModbusOk;  // Reports whether the ModBus connection to the VFD that controls the spindle is ok. Connect to spindle-vfd.modbus-ok.

  // Following error flags for each joint
  hal_bit_t *xFError;
  hal_bit_t *yFError;
  hal_bit_t *zFError;
  hal_bit_t *bFError;
  hal_bit_t *cFError;

  hal_bit_t *ignoreComErrors;

  // The user-request-enable pin should be connected
  // to iocontrol.0.user-request-enable and indicates
  // that the user wants to reset E-Stop (they clicked
  // a button in the UI).
  hal_bit_t *userRequestEnable;

  // The latched version of userRequestEnable. This will
  // stay high after the user clicks a button to reset
  // E-Stop until the motors have a chance to disable then
  // re-enable.
  // This is also an output that must be connected to
  // halui.estop.reset in order to clear the software
  // E-Stop condition in the case that the user releases
  // the physical E-Stop (the software E-Stop is cleared
  // via the python interface when clicking the E-Stop
  // button in the UI).
  hal_bit_t *userRequestedEnable;

  // Timer that restarts when the user clicks a button to
  // reset E-Stop. Is used to time various pulse times,
  // such as disabling/re-enabling the motors and turning on
  // the machine.
  hal_u32_t timeSinceEnable;

  hal_u32_t timeSinceEStop;

  // Timer that starts at initialization. Used to prevent
  // motor faults at start up, when they haven't been powered
  // on yet.
  hal_u32_t timeSinceStartUp;

  // Timer that starts once the physical E-Stop button has been
  // released. Used to make assumptions about motor and spindle
  // faults and to allow enough time to pass before attempting
  // to re-enable motors after powering them up.
  hal_u32_t timeSinceButtonRelease;

  // emcEnable is False when in E-Stop and True when not in E-stop.
  // Connect to iocontrol.0.emc-enable-in.
  hal_bit_t *emcEnable;

  // Connect to iocontrol.0.user-enable-out to trigger
  // faults internal to EMC.
  hal_bit_t *userEnable;

  // Output pin that controls whether the machine is on.
  // Connect to halui.machine.on
  hal_bit_t *machineOn;

  // Power is connected to a pin that can trigger the same relay as
  // the physical E-Stop button in order to cut power to the spindle
  // and axis motors. Currently unused.
  hal_bit_t *power;

  // Individual enable pins for each motor. Necessary to be able to disable/re-enable
  // motors after a motor fault. This will need to be revisited if we implement hard
  // stop homing as the motors will also need to be controlled by the homing routine
  // (see SOFT-455).
  hal_bit_t *xMotorEnable;
  hal_bit_t *yMotorEnable;
  hal_bit_t *zMotorEnable;
  hal_bit_t *bMotorEnable;
  hal_bit_t *cMotorEnable;

  hal_bit_t *unhome;
} data_t;

// Max time of the timer. Since we don't need the timer for very long
// this ensures we never overflow the timer.
#define MAX_TIME 6000

#define UNHOME_TIME 100

// Time when the machine-on pin should go high after a reset.
#define MACHINE_ON_TIME 1100

// How much time to give all the motors to get out of a fault state at start up or
// after releasing the physical E-Stop button.
// The motors report a fault when powered off, so we use various conditions to
// avoid reporting a fault in that condition, such as at startup and when the
// physical E-Stop button is pushed.
#define STARTUP_TIME 3000

// How long to disable the motors for after a reset. Will be re-enabled after
// this time elapses.
#define DISABLE_MOTOR_TIME 100

// Time that must elapse after the user clicks the E-Stop reset in the UI
// before we actually reset software E-Stop (this helps prevent reporting
// motor faults when the motors are still resetting).
#define RESET_TIME 1000

static data_t *data;

static const char *modname = "pro-estop";
static int comp_id;

static void update(void *arg, long period) {
  const hal_bit_t ignoreComErrors = *(data->ignoreComErrors);
  const hal_bit_t notIgnoreComErrors = !ignoreComErrors;

  const hal_bit_t xFault = *(data->xFault) && notIgnoreComErrors;
  const hal_bit_t yFault = *(data->yFault) && notIgnoreComErrors;
  const hal_bit_t zFault = *(data->zFault) && notIgnoreComErrors;
  const hal_bit_t bFault = *(data->bFault) && notIgnoreComErrors;
  const hal_bit_t cFault = *(data->cFault) && notIgnoreComErrors;

  const hal_bit_t xFError = *(data->xFError);
  const hal_bit_t yFError = *(data->yFError);
  const hal_bit_t zFError = *(data->zFError);
  const hal_bit_t bFError = *(data->bFError);
  const hal_bit_t cFError = *(data->cFError);

  const hal_bit_t button = *(data->button);
  const hal_s32_t spindleErrorCode = *(data->spindleErrorCode) && notIgnoreComErrors;
  const hal_bit_t spindleModbusOk = *(data->spindleModbusOk) || ignoreComErrors;
  const hal_u32_t timeSinceEnable = data->timeSinceEnable;
  const hal_bit_t userRequestEnable = *(data->userRequestEnable);
  const hal_bit_t userRequestedEnable = *(data->userRequestedEnable);

  const hal_bit_t xFaulted = data->xFaulted;
  const hal_bit_t yFaulted = data->yFaulted;
  const hal_bit_t zFaulted = data->zFaulted;
  const hal_bit_t bFaulted = data->bFaulted;
  const hal_bit_t cFaulted = data->cFaulted;

  const hal_bit_t xFErrored = data->xFErrored;
  const hal_bit_t yFErrored = data->yFErrored;
  const hal_bit_t zFErrored = data->zFErrored;
  const hal_bit_t bFErrored = data->bFErrored;
  const hal_bit_t cFErrored = data->cFErrored;

  const hal_bit_t spindleErroredWithCode = data->spindleErroredWithCode;
  const hal_bit_t spindleModbusNotOk = data->spindleModbusNotOk;
  const hal_bit_t buttonPushed = data->buttonPushed;
  const hal_bit_t buttonReleased = data->buttonReleased;

  // The motors and spindle VFD report errors when power is cut due to the physical E-Stop button being pressed.
  // To avoid reporting that there is a spindle or motor fault when the button is pressed, we use the following
  // conditions. 
  // Check that the button isn't pressed or was latched.
  // Check that enough time after start up has elapsed.
  // Check that enough time after a user initiated E-Stop reset has elapsed.
  const hal_bit_t preventFaultsFromButtonPushAndStartup = !button && 
                                                          !buttonPushed && 
                                                          data->timeSinceStartUp > STARTUP_TIME && 
                                                          data->timeSinceEnable > RESET_TIME &&
                                                          data->timeSinceButtonRelease > STARTUP_TIME;

  if(xFault && preventFaultsFromButtonPushAndStartup) {
    // Only report the fault when it first happens
    if(!xFaulted) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Motor X fault.");
    }
    data->xFaulted = true;
  }

  if(yFault && preventFaultsFromButtonPushAndStartup) {
    // Only report the fault when it first happens
    if(!yFaulted) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Motor Y fault.");
    }
    data->yFaulted = true;
  }

  if(zFault && preventFaultsFromButtonPushAndStartup) {
    if(!zFaulted) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Motor Z fault.");
    }
    data->zFaulted = true;
  }

  if(bFault && preventFaultsFromButtonPushAndStartup) {
    if(!bFaulted) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Motor B fault.");
    }
    data->bFaulted = true;
  }

  if(cFault && preventFaultsFromButtonPushAndStartup) {
    if(!cFaulted) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Motor C fault.");
    }
    data->cFaulted = true;
  }

  if(xFError) {
    if(!xFErrored) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: X following error.");
    }
    data->xFErrored = true;
  }

  if(yFError) {
    if(!yFErrored) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Y following error.");
    }
    data->yFErrored = true;
  }

  if(zFError) {
    if(!zFErrored) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Z following error.");
    }
    data->zFErrored = true;
  }

  if(bFError) {
    if(!bFErrored) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: B following error.");
    }
    data->bFErrored = true;
  }

  if(cFError) {
    if(!cFErrored) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: C following error.");
    }
    data->cFErrored = true;
  }

  if(spindleErrorCode != 0 && preventFaultsFromButtonPushAndStartup) {
    if(spindleErroredWithCode == 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Spindle error: code %d", spindleErrorCode);
    }
    data->spindleErroredWithCode = spindleErrorCode;
  }

  if(!spindleModbusOk && preventFaultsFromButtonPushAndStartup) {
    if(!spindleModbusNotOk) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop: Spindle communication error.");
    }
    data->spindleModbusNotOk = true;
  }

  if(button) {
    if(!buttonPushed) {
      rtapi_print_msg(RTAPI_MSG_ERR, "E-Stop button pressed.");
    }
    data->buttonPushed = true;
  } 

  if(buttonPushed && !button) {
    if(!buttonReleased) {
      data->timeSinceButtonRelease = 0;
    }
    data->buttonReleased = true;
  }

  *(data->unhome) = data->estopped && data->timeSinceEStop > UNHOME_TIME;

  // current fault state
  hal_bit_t fault = xFault ||
                    yFault ||
                    zFault ||
                    bFault ||
                    cFault ||
                    xFError ||
                    yFError ||
                    zFError ||
                    bFError ||
                    cFError ||
                    !spindleModbusOk ||
                    spindleErrorCode != 0 ||
                    button;

  // latched fault value
  hal_bit_t faulted = xFaulted ||
                      yFaulted ||
                      zFaulted ||
                      bFaulted ||
                      cFaulted ||
                      xFErrored ||
                      yFErrored ||
                      zFErrored ||
                      bFErrored ||
                      cFErrored ||
                      spindleModbusNotOk ||
                      spindleErroredWithCode != 0 ||
                      buttonPushed;

  // When user initiates an E-Stop reset
  if(!(*(data->userRequestedEnable)) && (userRequestEnable || (buttonReleased && data->timeSinceButtonRelease > STARTUP_TIME))) {
    // Latch the userRequestedEnable variable and reset our timer
    *(data->userRequestedEnable) = 1;
    data->timeSinceEnable = 0;
  }

  hal_bit_t reset = false;
  if(*(data->userRequestedEnable)) {
    // Once the user has requested to reset E-Stop, we disable
    // the motors and re-enable them to clear any fault conditions.
    if(data->timeSinceEnable < DISABLE_MOTOR_TIME) {
      *(data->xMotorEnable) = false;
      *(data->yMotorEnable) = false;
      *(data->zMotorEnable) = false;
      *(data->bMotorEnable) = false;
      *(data->cMotorEnable) = false;
    } else {
      *(data->xMotorEnable) = true;
      *(data->yMotorEnable) = true;
      *(data->zMotorEnable) = true;
      *(data->bMotorEnable) = true;
      *(data->cMotorEnable) = true;
    }

    // Once enough time has elapsed, actually reset the E-Stop.
    if(data->timeSinceEnable > RESET_TIME) {
      // unlatch faults, they'll relatch if the fault continues
      // so the errors will be reported again in response to the
      // user attempting to reset e-stop
      data->xFaulted = false;
      data->yFaulted = false;
      data->zFaulted = false;
      data->bFaulted = false;
      data->cFaulted = false;

      data->xFErrored = false;
      data->yFErrored = false;
      data->zFErrored = false;
      data->bFErrored = false;
      data->cFErrored = false;

      data->spindleErroredWithCode = 0;
      data->spindleModbusNotOk = false;
      data->buttonPushed = false;
      data->buttonReleased = false;

      data->estopped = false;
      *(data->userRequestedEnable) = false;

      reset = true;
    }
  }

  // prevent potentially overflowing our timer variable
  if(data->timeSinceButtonRelease <= MAX_TIME) {
    data->timeSinceButtonRelease += 1;
  }

  // prevent potentially overflowing our timer variable
  if(data->timeSinceEnable <= MAX_TIME) {
    data->timeSinceEnable += 1;
  }

  // prevent potentially overflowing our timer variable
  if(data->timeSinceStartUp <= MAX_TIME) {
    data->timeSinceStartUp += 1;
  }

  if(data->timeSinceEStop <= MAX_TIME) {
    data->timeSinceEStop += 1;
  }

  data->estop = !(!fault && *(data->userEnable) && (!faulted || (faulted && reset)));

  if(data->estop && !data->estopped) {
    data->timeSinceEStop = 0;
    data->estopped = true;
  }

  *(data->emcEnable) = !data->estop;

  // Delay turning the machine on for a short period of time after resetting the software E-Stop.
  // machineOn should be connected to halui.machine.on, which will drive halui.machine.is-on high,
  // which is required in order to home or do anything CNC-wise. It doesn't seem to always take if
  // it's toggled on at the same time as emcEnable, so we add a small delay to ensure we properly
  // set the machine state to on.
  *(data->machineOn) = *(data->emcEnable) && data->timeSinceEnable > MACHINE_ON_TIME;
}

int rtapi_app_main(void) {
  int retval;
  comp_id = hal_init(modname);
  if(comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
    return -1;
  }

  data = hal_malloc(sizeof(data_t));

  PIN(bit, HAL_IN, xFault, x-fault);
  PIN(bit, HAL_IN, yFault, y-fault);
  PIN(bit, HAL_IN, zFault, z-fault);
  PIN(bit, HAL_IN, bFault, b-fault);
  PIN(bit, HAL_IN, cFault, c-fault);

  PIN(bit, HAL_IN, xFError, x-f-error);
  PIN(bit, HAL_IN, yFError, y-f-error);
  PIN(bit, HAL_IN, zFError, z-f-error);
  PIN(bit, HAL_IN, bFError, b-f-error);
  PIN(bit, HAL_IN, cFError, c-f-error);

  PIN(bit, HAL_IN, ignoreComErrors, ignore-com-errors);

  PIN(bit, HAL_IN, button, button);
  PIN(s32, HAL_IN, spindleErrorCode, spindle-error-code);
  PIN(bit, HAL_IN, spindleModbusOk, spindle-modbus-ok);

  PIN(bit, HAL_IN, userRequestEnable, user-request-enable);
  PIN(bit, HAL_OUT, userRequestedEnable, user-requested-enable);

  PIN(bit, HAL_OUT, emcEnable, emc-enable);
  PIN(bit, HAL_IN, userEnable, user-enable);

  PIN(bit, HAL_OUT, power, power);
  PIN(bit, HAL_OUT, machineOn, machine-on);
  PIN(bit, HAL_OUT, xMotorEnable, x-motor-enable);
  PIN(bit, HAL_OUT, yMotorEnable, y-motor-enable);
  PIN(bit, HAL_OUT, zMotorEnable, z-motor-enable);
  PIN(bit, HAL_OUT, bMotorEnable, b-motor-enable);
  PIN(bit, HAL_OUT, cMotorEnable, c-motor-enable);
  PIN(bit, HAL_OUT, unhome, unhome);

  *(data->xFault) = 0;
  *(data->yFault) = 0;
  *(data->zFault) = 0;
  *(data->bFault) = 0;
  *(data->cFault) = 0;

  *(data->xFError) = 0;
  *(data->yFError) = 0;
  *(data->zFError) = 0;
  *(data->bFError) = 0;
  *(data->cFError) = 0;

  *(data->button) = 0;
  *(data->spindleErrorCode) = 0;
  *(data->spindleModbusOk) = 1;

  *(data->emcEnable) = 0;

  *(data->power) = 1;
  *(data->xMotorEnable) = 1;
  *(data->yMotorEnable) = 1;
  *(data->zMotorEnable) = 1;
  *(data->bMotorEnable) = 1;
  *(data->cMotorEnable) = 1;
  *(data->machineOn) = 0;

  *(data->ignoreComErrors) = 0;

  data->xFaulted = 0;
  data->yFaulted = 0;
  data->zFaulted = 0;
  data->bFaulted = 0;
  data->cFaulted = 0;

  data->xFErrored = 0;
  data->yFErrored = 0;
  data->zFErrored = 0;
  data->bFErrored = 0;
  data->cFErrored = 0;

  data->estop = 0;
  data->estopped = 0;

  data->timeSinceEStop = 0;
  data->timeSinceEnable = 0;
  data->timeSinceStartUp = 0;

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
