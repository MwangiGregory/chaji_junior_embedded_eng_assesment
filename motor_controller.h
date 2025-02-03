#include "Arduino.h"
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "pin_out.h"

#define MOTOR_OFF_SPEED 0

#define MOTOR_ON true
#define MOTOR_OFF false
#define MOTOR_SPEED_PWM_SIG 200

#define MAX_ADC_VALUE 1023.0
#define MAX_ADC_VOLTAGE 5.0
#define SHUNT_RESISTANCE 1.0

#define MAX_SAFE_MOTOR_CURRENT 2.0 // 2A

class MotorController
{
  typedef uint8_t pin_t;

public:
  MotorController(pin_t motor_current_read_pin, pin_t motor_speed_control_pin, pin_t led_pin)
        : m_motor_current_pin{motor_current_read_pin}, m_motor_speed_pin{motor_speed_control_pin}, m_led_pin{led_pin}
  {
    pinMode(m_motor_current_pin, INPUT);
    pinMode(m_motor_speed_pin, OUTPUT);
    pinMode(m_led_pin, OUTPUT);

    digitalWrite(m_led_pin, LOW);
    set_motor_state(MOTOR_OFF);
    set_motor_speed(MOTOR_OFF_SPEED);
    run_motor();
  }

  void set_motor_state(bool motor_on) { m_motor_state = motor_on; }
  void set_motor_speed(uint8_t motor_speed) { m_motor_speed = motor_speed; }
  bool check_motor_overcurrent()
  {
    float motor_current = read_motor_current();
    if (motor_current < 0.0)
      return true;
    else if (motor_current >= MAX_SAFE_MOTOR_CURRENT)
      return true;
    
    return false;
  }
  void run_motor()
  {
    if (check_motor_overcurrent() == true)
    {
      set_motor_state(MOTOR_OFF);
      digitalWrite(m_led_pin, HIGH);
    }
    else
      digitalWrite(m_led_pin, LOW);
  
    if (m_motor_state == MOTOR_ON)
      analogWrite(m_motor_speed_pin, m_motor_speed);
    else if (m_motor_state == MOTOR_OFF)
      analogWrite(m_motor_speed_pin, MOTOR_OFF_SPEED);
  }
  
private:
  float read_motor_current()
  {
    int raw_adc_val = analogRead(m_motor_current_pin);
    float shunt_resistor_voltage = (raw_adc_val / MAX_ADC_VALUE) * MAX_ADC_VOLTAGE;
    float shunt_resistor_current = shunt_resistor_voltage / SHUNT_RESISTANCE; // voltage across resistor divided by resistance
    return shunt_resistor_current;
  }

  // Make object non copyable
  MotorController(const MotorController&) = delete;
  MotorController& operator=(const MotorController&) = delete;

  uint8_t m_motor_speed {};
  bool m_motor_state {MOTOR_OFF};
  pin_t m_motor_current_pin;
  pin_t m_motor_speed_pin;
  pin_t m_led_pin;
};

#endif /* MOTOR_CONTROLLER_H */