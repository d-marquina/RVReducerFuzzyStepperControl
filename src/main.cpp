/*
Arduino Espressiff version: 2.0.6
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include "driver/pcnt.h"
#include "driver/mcpwm.h"

HardwareSerial ESP32Serial1(1);
#define EN_PIN 5                               // Enable
#define DIR_PIN 4                              // Direction
#define STEP_PIN 2                             // Step
#define R_SENSE 0.11f                          // SilentStepStick series use 0.11
TMC2208Stepper driver(&ESP32Serial1, R_SENSE); // Hardware Serial
bool stepper_dir = false;                     // Positive

int led_pin = 21;

// setting PWM properties
int step_freq = 2000; // 2kHz
const int ledChannel = 0;

// PCNT unit parameters
int out_enc_A = 18;
int out_enc_B = 19;
int16_t out_enc_pulses = 10;
float out_enc_pos = 0;

// MCPWM Capture
uint32_t cap_tick_0 = 0;
uint32_t cap_tick_1 = 0;
uint32_t cap_n_ticks = 0;
float out_sp_rpm = 0;
bool out_sp_dir = false; // CCW, Positive
static uint8_t out_enc_st_code = 0;
static uint16_t out_enc_st_store=0;
static int8_t out_enc_st_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

void update_out_sp_rpm(){
  // Calculate
  out_sp_rpm = 400000.0/cap_n_ticks; // Max: 5 RPM

  // Approximate to Zero
  if(out_sp_rpm < 0.1){ // Threshold: 0.1 RPM
    out_sp_rpm = 0;
    return;
  }

  // Apply direction
  if (out_sp_dir) out_sp_rpm = -1*out_sp_rpm;

  return;
}

static bool mcpwm_isr_function(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata, void *arg) {
  BaseType_t high_task_wakeup = pdFALSE;

  // Decode direction
  out_enc_st_code <<= 2;
  if (digitalRead(out_enc_A)) out_enc_st_code |= 0x01;
  if (digitalRead(out_enc_B)) out_enc_st_code |= 0x02;
  out_enc_st_code &= 0x0f;

   // If valid then store as 16 bit data.
   if (out_enc_st_table[out_enc_st_code] ) {
    out_enc_st_store <<= 4;
    out_enc_st_store |= out_enc_st_code;
    if ((out_enc_st_store&0xff)==0x2b) out_sp_dir = true; // CW, negative
    if ((out_enc_st_store&0xff)==0x17) out_sp_dir = false; // CCW, positive

    // Measure number of ticks between 2 consecutive pulses, from different signals
    cap_tick_1 = edata->cap_value;
    cap_n_ticks = cap_tick_1 - cap_tick_0;
    cap_tick_0 = edata->cap_value;//*/
   }
  
  return high_task_wakeup == pdTRUE;
}

void setup(){
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);

  ESP32Serial1.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(EN_PIN, OUTPUT);
  ledcAttachPin(STEP_PIN, ledChannel);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  // Configure PCNT Unit 0
  pcnt_config_t pcnt_u0_ch0 = {
      .pulse_gpio_num = out_enc_A,
      .ctrl_gpio_num = out_enc_B,      
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = 12000,
      .counter_l_lim = -12000,
      .unit = PCNT_UNIT_0,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_config_t pcnt_u0_ch1 = {
      .pulse_gpio_num = out_enc_B,
      .ctrl_gpio_num = out_enc_A,   
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = 12000,
      .counter_l_lim = -12000,
      .unit = PCNT_UNIT_0,
      .channel = PCNT_CHANNEL_1,
  };
  pcnt_unit_config(&pcnt_u0_ch0);
  pcnt_unit_config(&pcnt_u0_ch1);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // Configure MCPWM Capture
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, out_enc_A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, out_enc_B);
  mcpwm_capture_config_t mcpwm0_cap0 = {
    .cap_edge = MCPWM_BOTH_EDGE,
    .cap_prescale = 1,
    .capture_cb = mcpwm_isr_function,
    .user_data = NULL
  };
  mcpwm_capture_config_t mcpwm0_cap1 = {
    .cap_edge = MCPWM_BOTH_EDGE,
    .cap_prescale = 1,
    .capture_cb = mcpwm_isr_function,
    .user_data = NULL
  };
  mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &mcpwm0_cap0);
  mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, &mcpwm0_cap1);

  driver.begin();           // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);           // Enables driver in software
  driver.rms_current(1000); // Set motor RMS current
  driver.microsteps(4);     // Set microsteps to 1/16th
  driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);   // Needed for stealthChop
}

void loop(){
  // Linear Ramp using LedCPWM
  /*for (int i = 0; i < 100; i++){
    if (i < 25){
      step_freq = 2000*i/25;
      ledcWriteTone(ledChannel, step_freq);
    } else if (i < 75){
      step_freq = 2000;
      ledcWriteTone(ledChannel, step_freq);
    } else {
      step_freq = 2000*(100-i)/25;
      ledcWriteTone(ledChannel, step_freq);
    }
    delay(100);
  }
  ledcWrite(ledChannel, 0);
  delay(4000);*/

  // Sinusoidal Ramp using LedCPWM
  float step_freq_f = 0;
  char step_freq_c[20];
  char pulses_c[20];
  char out_sp_rpm_c[20];

  for (int i = 0; i < 100; i++){
    if (i < 30){
      step_freq = i - 15;
      step_freq_f = 0.104719333 * step_freq;
      step_freq = int(1000 * sin(step_freq_f)) + 1000;
      ledcWriteTone(ledChannel, step_freq);
    } else if (i < 70){
      step_freq = 2000;
      ledcWriteTone(ledChannel, step_freq);
    } else {
      step_freq = i + 5;
      step_freq_f = 0.104719333 * step_freq;
      step_freq = int(1000 * sin(step_freq_f)) + 1000;
      ledcWriteTone(ledChannel, step_freq);
    }
    // Send data
    //Serial.println(itoa(step_freq, step_freq_c, 10));
    //pcnt_get_counter_value(PCNT_UNIT_0, &out_enc_pulses);
    //Serial.println(itoa(out_enc_pulses, pulses_c, 10));
    update_out_sp_rpm();
    //out_sp_rpm = 400000.0/cap_n_ticks;
    dtostrf(out_sp_rpm, 6, 3, out_sp_rpm_c);
    Serial.println(out_sp_rpm_c);
    delay(100);
  }
  ledcWrite(ledChannel, 0);

  for (int i = 0; i < 50; i++){
    // Send data
    //Serial.println("0"); // step_freq = 0
    //pcnt_get_counter_value(PCNT_UNIT_0, &out_enc_pulses);
    //Serial.println(itoa(out_enc_pulses, pulses_c, 10));
    update_out_sp_rpm();
    //out_sp_rpm = 400000.0/cap_n_ticks;
    dtostrf(out_sp_rpm, 6, 3, out_sp_rpm_c);
    Serial.println(out_sp_rpm_c);
    delay(80);
  }

  stepper_dir = !stepper_dir;
  driver.shaft(stepper_dir);

  if (stepper_dir){
    digitalWrite(led_pin, HIGH);
  } else {
    digitalWrite(led_pin, LOW);
  }
}


