#ifndef DISPATCH_CONTROLLER_H
#define DISPATCH_CONTROLLER_H

#include "Rtd_Handler.h"
#include "Can_Node_Handler.h"
#include "Bms_Handler.h"
#include "Motor_Handler.h"
#include "Current_Sense_Handler.h"

class Dispatch_Controller {
  public:
    static Dispatch_Controller& getInstance();
    void begin();
    void disable();
    void enable();
    void dispatch();
    void handleFaultPins();
    bool isEnabled();
    void requestMotorHeartbeat();
    void requestLeftMotorUpdates();
    void requestRightMotorUpdates();
  private:
    Dispatch_Controller();
    void initializeFaultPins();
    bool handleSingleFaultPin(int pin, String pinName);
    static Dispatch_Controller *instance;
    Rtd_Handler rtd_handler;
    Can_Node_Handler can_node_handler;
    Bms_Handler bms_handler;
    Motor_Handler motor_handler;
    Current_Sense_Handler current_sense_handler;
    bool begun;
    bool enabled;
};

Dispatch_Controller& Dispatcher();

#endif // DISPATCH_CONTROLLER_H
