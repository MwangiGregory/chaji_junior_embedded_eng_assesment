#include "motor_controller.h"
#include "pin_out.h"

#define BUTTON_IS_PRESSED LOW

bool is_start_stop_pressed()
{
  return (bool) digitalRead(PUSH_BUTTON_PIN);  // debounce done in hardware
}

static MotorController motor_controller {MOTOR_CURRENT_READ_PIN, MOTOR_SPEED_CONTROL_PIN};

void setup() {
  
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);

  motor_controller.set_motor_state(MOTOR_OFF);
  motor_controller.set_motor_speed(MOTOR_OFF_SPEED);
}

void loop() {

  static bool motor_state = MOTOR_OFF;

  motor_controller.run_motor();

  if (is_start_stop_pressed() == BUTTON_IS_PRESSED)
  {
    motor_state = !motor_state;
    motor_controller.set_motor_state(motor_state);
  }
}


