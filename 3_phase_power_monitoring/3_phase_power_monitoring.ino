#define  CURRENT_SENSE_PIN 19
#define VOLTAGE_SENSE_PIN 20

#define NUM_OF_SAMPLES 500
#define ADC_RESOLUTION 1023.0
#define VOLTAGE_REFERENCE 3.3
#define SHUNT_RESISTANCE 1.0
#define CURRENT_SENSE_TURNS_RATIO (100.0 / 1.0) // primary : secondary
#define VOLTAGE_SENSE_TURNS_RATIO (2000.0 / 200.0) // primary : secondary

#define SERIAL_BAUD 115200

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(CURRENT_SENSE_PIN, INPUT);
  pinMode(VOLTAGE_SENSE_PIN, INPUT);
}

void loop() {
  double sum_of_current_squares {};
  double sum_of_voltage_squares {};

  for (int i = 0; i < NUM_OF_SAMPLES; i++)
  {
    float mains_current = get_instantaneous_current(CURRENT_SENSE_PIN);
    float mains_voltage = get_instantaneous_voltage(VOLTAGE_SENSE_PIN);

    sum_of_current_squares += powf(mains_current, 2);
    sum_of_voltage_squares += powf(mains_voltage, 2);
  }

  double rms_mains_current = sqrt(sum_of_current_squares / NUM_OF_SAMPLES);
  double rms_mains_voltage = sqrt(sum_of_voltage_squares / NUM_OF_SAMPLES);
  double real_power = rms_mains_voltage * rms_mains_current;

  char text_output[128] {};
  sprintf(
    text_output,
    "RMS VOLTAGE: %f VOLTS, RMS CURRENT: %f AMPS, REAL POWER: %f WATTS\n",
    rms_mains_voltage,
    rms_mains_current,
    real_power);
  
  Serial.print(text_output);
}

float get_instantaneous_current(uint8_t current_sense_pin)
{
  int raw_current_adc = analogRead(current_sense_pin);
  float voltage = (raw_current_adc / ADC_RESOLUTION) * VOLTAGE_REFERENCE;
  float secondary_current = voltage / SHUNT_RESISTANCE;
  float primary_current = secondary_current * CURRENT_SENSE_TURNS_RATIO;
  return primary_current;
}

float get_instantaneous_voltage(uint8_t voltage_sense_pin)
{
// voltage sense voltage divider. Resistor names are reference from the schematic
#define R3_RESISTANCE 20 // 20k
#define R7_RESISTANCE 10 // 10k

  int raw_voltage_adc = analogRead(voltage_sense_pin);
  float voltage_divider_output = (raw_voltage_adc / ADC_RESOLUTION) * VOLTAGE_REFERENCE;
  float secondary_voltage = VOLTAGE_REFERENCE * (R3_RESISTANCE + R7_RESISTANCE) / R7_RESISTANCE; // convert output of volatage divider back to undivided volatage
  float primary_voltage = secondary_voltage * VOLTAGE_SENSE_TURNS_RATIO;
  return primary_voltage;
}