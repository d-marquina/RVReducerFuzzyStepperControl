#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>

HardwareSerial ESP32Serial1(1);

#define EN_PIN           5 // Enable
#define DIR_PIN          4 // Direction
#define STEP_PIN         2 // Step
//#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define R_SENSE 0.11f // SilentStepStick series use 0.11
TMC2208Stepper driver(&ESP32Serial1, R_SENSE); // Hardware Serial
bool stepper_dir = false;

int led_pin = 19;

// setting PWM properties
int step_freq = 2000; // 2kHz
const int ledChannel = 0;
//const int resolution = 8;

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);
  
  ESP32Serial1.begin(115200, SERIAL_8N1, 16, 17);

  // configure LED PWM functionalitites
  //ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(STEP_PIN, ledChannel);

  pinMode(EN_PIN, OUTPUT);
  //pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  driver.begin();          // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);          // Enables driver in software
  driver.rms_current(1000); // Set motor RMS current
  driver.microsteps(4);   // Set microsteps to 1/16th

  // driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true); // Needed for stealthChop
}

void loop() {
  /*Serial.println("Hello World");
  digitalWrite(led_pin, HIGH);
  delay(2000);
  digitalWrite(led_pin, LOW);
  delay(2000);*/

  // Run 400 steps and switch direction in software
  /*for (uint16_t i = 24000; i>0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(250);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(250);
  }
  delay(4000);*/

  /*ledcWriteTone(ledChannel, 2000);
  delay(2000);
  ledcWriteTone(ledChannel, 1000);
  delay(2000);
  ledcWrite(ledChannel, 0);
  delay(4000);*/

  for (int i = 0; i < 100; i++){
    if (i < 25){
      step_freq = 2000*i/25;
      ledcWriteTone(ledChannel, step_freq);
    } else if (i < 75){
      step_freq = 2000;
      ledcWriteTone(ledChannel, step_freq);
    } else{
      step_freq = 2000*(100-i)/25;
      ledcWriteTone(ledChannel, step_freq);
    }
    delay(100);
  }
  ledcWrite(ledChannel, 0);
  delay(4000);

  stepper_dir = !stepper_dir;
  driver.shaft(stepper_dir);
  
  if (stepper_dir){
    digitalWrite(led_pin, HIGH);
  } else {
    digitalWrite(led_pin, LOW);
  }
  
}