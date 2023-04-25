/*
Arduino Espressiff version: 2.0.6
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>

#include "driver/pcnt.h"
#include "driver/mcpwm.h"

/*#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/rtc.h"
#include "esp_log.h"

const static char *TAG = "Custom ESP32";*/

HardwareSerial ESP32Serial1(1);

#define EN_PIN 5                               // Enable
#define DIR_PIN 4                              // Direction
#define STEP_PIN 2                             // Step
#define R_SENSE 0.11f                          // SilentStepStick series use 0.11
TMC2208Stepper driver(&ESP32Serial1, R_SENSE); // Hardware Serial
bool stepper_dir = false;

int led_pin = 21;

// setting PWM properties
int step_freq = 2000; // 2kHz
const int ledChannel = 0;

// PCNT unit parameters
int enc_A = 18;
int enc_B = 19;
int16_t pulses = 10;

// MCPWM Capture
uint32_t cap_val_begin_of_sample = 0;
uint32_t cap_val_end_of_sample = 0;
uint32_t n_pulses = 0;
float vel_rpm = 0;
//static xQueueHandle cap_queue;

char *ftoa(double f, char *a){
  // Convert float to ascii!
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  int desimal = abs((int)((f - heiltal) * 100));  // int is enough for 2 digits
  if (desimal< 10)  //are there leading zeros?
    { *a='0'; a++; }
  itoa(desimal, a, 10);
  return ret;
}

static bool mcpwm_isr_function(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata,
                                  void *arg) {
    BaseType_t high_task_wakeup = pdFALSE;

    // Measure number of ticks during high pulse
    if (edata->cap_edge == MCPWM_POS_EDGE) {
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        n_pulses = cap_val_end_of_sample - cap_val_begin_of_sample;
    }

    // Measure number of ticks between pulses 
    // TODO

    return high_task_wakeup == pdTRUE;
}

void setup(){
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);

  ESP32Serial1.begin(115200, SERIAL_8N1, 16, 17);

  ledcAttachPin(STEP_PIN, ledChannel);
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  // Configure PCNT Unit 0
  pcnt_config_t pcnt_u0_ch0 = {
      .pulse_gpio_num = enc_A,
      .ctrl_gpio_num = enc_B,      
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
      .pulse_gpio_num = enc_B,
      .ctrl_gpio_num = enc_A,   
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
  /*cap_queue = xQueueCreate(1, sizeof(uint32_t));
  if (cap_queue == NULL) {
    ESP_LOGE(TAG, "failed to alloc cap_queue");
    return;
  }*/
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, enc_A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, enc_B);
  mcpwm_capture_config_t mcpwm0_cap0 = {
    .cap_edge = MCPWM_BOTH_EDGE,
    .cap_prescale = 1,
    .capture_cb = mcpwm_isr_function,
    .user_data = NULL
  };
  /*mcpwm_capture_config_t mcpwm0_cap1 = {
    .cap_edge = MCPWM_BOTH_EDGE,
    .cap_prescale = 1,
    .capture_cb = mcpwm_isr_function,
    .user_data = NULL
  };*/
  mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &mcpwm0_cap0);
  //mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, &mcpwm0_cap1);

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
    } else{
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
  char vel_rpm_c[20];

  for (int i = 0; i < 100; i++){
    float step_freq_f = 0;
    char step_freq_c[20];
    char pulses_c[20];
    if (i < 30){
      step_freq = i - 15;
      step_freq_f = 0.104719333 * step_freq;
      step_freq = int(1000 * sin(step_freq_f)) + 1000;
      ledcWriteTone(ledChannel, step_freq);
    } else if (i < 70){
      step_freq = 2000;
      ledcWriteTone(ledChannel, step_freq);
    } else{
      step_freq = i + 5;
      step_freq_f = 0.104719333 * step_freq;
      step_freq = int(1000 * sin(step_freq_f)) + 1000;
      ledcWriteTone(ledChannel, step_freq);
    }
    //Serial.println(itoa(step_freq, step_freq_c, 10));
    //pcnt_get_counter_value(PCNT_UNIT_0, &pulses);
    //Serial.println(itoa(pulses, pulses_c, 10));
    //xQueueReceive(cap_queue, &local_pulse_ti, portMAX_DELAY);
    vel_rpm = 3200000.0/n_pulses;
    Serial.println(ftoa(vel_rpm, vel_rpm_c));
    delay(100);
  }
  ledcWrite(ledChannel, 0);

  for (int i = 0; i < 100; i++){
    //Serial.println("0"); // step_freq = 0
    //pcnt_get_counter_value(PCNT_UNIT_0, &pulses);
    //Serial.println(itoa(pulses, pulses_c, 10));
    //xQueueReceive(cap_queue, &local_pulse_ti, portMAX_DELAY);
    vel_rpm = 3200000.0/n_pulses;
    Serial.println(ftoa(vel_rpm, vel_rpm_c));
    delay(40);
  }

  stepper_dir = !stepper_dir;
  driver.shaft(stepper_dir);

  if (stepper_dir){
    digitalWrite(led_pin, HIGH);
  }else{
    digitalWrite(led_pin, LOW);
  }
}


