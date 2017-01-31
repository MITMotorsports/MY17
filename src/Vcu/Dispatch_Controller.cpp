#include "Dispatch_Controller.h"

//Magic timing library stuff
#include <SoftTimer.h>
#include <DelayRun.h>
#include <Debouncer.h>

#include "Can_Controller.h"
#include "Rtd_Controller.h"
#include "Store_Controller.h"
#include "Logger.h"

const int BLINK_LIGHT = 0;
const int ENABLE_LIGHT = 1;
const int SHUTDOWN_LIGHT = 2;

const uint16_t PRECHARGE_DELAY = 15000;

Dispatch_Controller::Dispatch_Controller()
: rtd_handler(Rtd_Handler()),
  can_node_handler(Can_Node_Handler()),
  bms_handler(Bms_Handler()),
  motor_handler(Motor_Handler()),
  current_sense_handler(Current_Sense_Handler()),
  begun(false),
  enabled(false)
{

}

// Handle messages as fast as possible
void dispatchPointer(Task*) {
  Dispatcher().dispatch();
}
Task stepTask(0, dispatchPointer);

// Check for faults at 10Hz
void checkFaults(Task*) {
  Dispatcher().handleFaultPins();
}
Task checkFaultsTask(100, checkFaults);

// Request message from both motors
void requestMotorHeartbeat(Task*) {
  Dispatcher().requestMotorHeartbeat();
}
Task heartbeatTask(100, requestMotorHeartbeat);

bool requestPermanentUpdatesLeft(Task*) {
  Dispatcher().requestLeftMotorUpdates();
  return false;
}
DelayRun requestLeftMotorUpdatesTask(50, requestPermanentUpdatesLeft);

bool requestPermanentUpdatesRight(Task*) {
  Dispatcher().requestRightMotorUpdates();
  return false;
}
DelayRun requestRightMotorUpdatesTask(100, requestPermanentUpdatesRight);

bool blinkDashLight(Task*) {
  if (!Dispatcher().isEnabled()) {
    // Only send light if not enabled
    Frame dashMessage = { .id=VCU_ID, .body={BLINK_LIGHT}, .len=1};
    CAN().write(dashMessage);
  }
  return false;
}
// Wait to blink light until precharge has finished
DelayRun requestBlinkDashLight(PRECHARGE_DELAY, blinkDashLight);

void Dispatch_Controller::requestMotorHeartbeat() {
  bool bothMcOn =
    Store().readMotorResponse(Store().RightMotor) &&
    Store().readMotorResponse(Store().LeftMotor);

  if (!bothMcOn) {
    motor_handler.requestHeartbeat();
  }
  else {
    SoftTimer.remove(&heartbeatTask);
    SoftTimer.add(&requestLeftMotorUpdatesTask);
    SoftTimer.add(&requestRightMotorUpdatesTask);
  }
}

void Dispatch_Controller::requestLeftMotorUpdates() {
  motor_handler.requestPermanentUpdates(LEFT_MOTOR_REQUEST_ID);
}
void Dispatch_Controller::requestRightMotorUpdates() {
  motor_handler.requestPermanentUpdates(RIGHT_MOTOR_REQUEST_ID);
}

void Dispatch_Controller::begin() {
  //Make idempotent
  if(begun) {
    return;
  }
  begun = true;

  // Initialize controllers
  CAN().begin();
  RTD().begin();

  // Initialize handlers
  rtd_handler.begin();
  can_node_handler.begin();
  bms_handler.begin();
  motor_handler.begin();

  initializeFaultPins();

  // Start event loop
  SoftTimer.add(&stepTask);
  SoftTimer.add(&checkFaultsTask);
  // Start MC requests
  SoftTimer.add(&heartbeatTask);

  Computer().logOne("vehicle_power_on");
  Onboard().logOne("vehicle_power_on");
  Xbee().logOne("vehicle_power_on");

}

// Must define instance prior to use
Dispatch_Controller* Dispatch_Controller::instance = NULL;

Dispatch_Controller& Dispatch_Controller::getInstance() {
  if (!instance) {
    instance = new Dispatch_Controller();
    instance->begin();
  }
  return *instance;
}

Dispatch_Controller& Dispatcher() {
  return Dispatch_Controller::getInstance();
}

bool Dispatch_Controller::isEnabled() {
  return enabled;
}

void Dispatch_Controller::disable() {
  // Force idempotency
  if(!enabled) {
    return;
  }
  enabled = false;

  // Actually disable
  RTD().disable();

  Frame disableMessage = { .id=VCU_ID, .body={BLINK_LIGHT}, .len=1};
  CAN().write(disableMessage);

  Computer().logOne("vehicle_disabled_or_shutdown");
  Onboard().logOne("vehicle_disabled_or_shutdown");
}

void Dispatch_Controller::enable() {
  // Force idempotency
  if(enabled) {
    return;
  }
  enabled = true;

  // Actually enable
  RTD().enable();

  // Notify listeners of enable
  Frame enableMessage = { .id=VCU_ID, .body={ENABLE_LIGHT}, .len=1};
  CAN().write(enableMessage);

  // TODO remove
  requestRightMotorUpdates();
  requestLeftMotorUpdates();

  Computer().logOne("vehicle_enabled");
  Onboard().logOne("vehicle_enabled");
}

void Dispatch_Controller::dispatch() {
  // If no message, break early
  while(CAN().msgAvailable()) {
    Frame frame = CAN().read();

    // Send message to each handler
    rtd_handler.handleMessage(frame);
    bms_handler.handleMessage(frame);
    can_node_handler.handleMessage(frame);
    motor_handler.handleMessage(frame);
    current_sense_handler.handleMessage(frame);
  }
}

void Dispatch_Controller::handleFaultPins() {
  bool bmsFault = handleSingleFaultPin(BMS_IN, "BMS");
  bool imdFault = handleSingleFaultPin(IMD_IN, "IMD");
  bool tempFault = handleSingleFaultPin(TEMP_SENSE_IN, "TEMP_SENSE");
  bool stopFault = handleSingleFaultPin(STOP_BUTTON_IN, "STOP_BUTTON");
  bool bmsPowerFault = handleSingleFaultPin(BMS_POWERED_IN, "BMS_NOT_POWERED");

  bool hasFault = bmsFault || imdFault || tempFault || stopFault || bmsPowerFault;

  bool prevHasFault = Store().readHasFault();
  if (hasFault && !prevHasFault) {
    // Car has shutdown :(
    // First logically disable
    Dispatcher().disable();

    // Then turn light off
    Frame dashMessage = { .id=VCU_ID, .body={SHUTDOWN_LIGHT}, .len=1};
    CAN().write(dashMessage);

    // Cancel precharge blink just in case
    SoftTimer.remove(&requestBlinkDashLight);
  } else if (!hasFault && prevHasFault) {
    // Tractive voltage is now live!

    // Set a timer to blink light after precharge
    SoftTimer.remove(&requestBlinkDashLight);
    requestBlinkDashLight.startDelayed();
    SoftTimer.add(&requestBlinkDashLight);
  }
  Store().logHasFault(hasFault);
}

bool Dispatch_Controller::handleSingleFaultPin(int pin, String pinName) {
  int fault = digitalRead(pin);
  if (fault == LOW) {
    Onboard().logTwo("LATCHED_FAULT", pinName);
    return true;
  }
  return false;
}

void Dispatch_Controller::initializeFaultPins() {
  pinMode(BMS_IN, INPUT);
  pinMode(IMD_IN, INPUT);
  pinMode(VCU_IN, INPUT);
  pinMode(TEMP_SENSE_IN, INPUT);
  pinMode(STOP_BUTTON_IN, INPUT);
  pinMode(BMS_POWERED_IN, INPUT);
}
