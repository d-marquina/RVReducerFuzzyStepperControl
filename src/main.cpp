/*
Arduino Espressiff version: 2.0.6
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include "driver/pcnt.h"
#include "driver/mcpwm.h"
#include <AS5048A.h>

HardwareSerial ESP32Serial1(1);
#define DIR_PIN 2                              // Direction
#define STEP_PIN 4                             // Step
#define EN_PIN 5                               // Enable
#define R_SENSE 0.11f                          // SilentStepStick series use 0.11
TMC2208Stepper driver(&ESP32Serial1, R_SENSE); // Hardware Serial
bool stepper_dir = false;                     // Positive

int led_pin = 19;

// setting PWM properties
int step_freq = 2000; // 2kHz
const int ledChannel = 0;

// PCNT unit parameters
int out_enc_A = 35;
int out_enc_B = 34;
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

// SPI Pins
int8_t VSPI_MISO = 26;
int8_t VSPI_SCLK = 25;
int8_t VSPI_SS = 33;
int8_t VSPI_MOSI = 32;
AS5048A stepper_enc(VSPI, VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS, false);
float stepper_angle_dg = 0;
float stepper_sp_dgps = 0;
float stepper_last_angle_dg = 0;
float st_enc_current_dg = 0;
float st_enc_last_dg = 0;
int16_t st_enc_full_rot = 0;
int16_t stepper_enc_raw = 0;//*/

// Multicore
TaskHandle_t Task1;

// Custom Functions

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

void debugAS5048A(){
  float local_stepper_enc_dg = 0;
  char local_stepper_enc_dg_c[20];//*/

  // Debugging AS5048
  Serial.println("ERRORS==============================");
  Serial.println(stepper_enc.getErrors());
  Serial.println("DIAGNOSTICS=========================");
  Serial.println(stepper_enc.getDiagnostic());
  Serial.println("LECTURE=============================");
  local_stepper_enc_dg = stepper_enc.getRotationInDegrees();
  dtostrf(local_stepper_enc_dg, 6, 3, local_stepper_enc_dg_c);
  Serial.printf("Angle: ");
  Serial.println(local_stepper_enc_dg_c);

  return;
}

void update_stepper_angle_dg(){
  st_enc_current_dg = stepper_enc.getRotationInDegrees();

  if (st_enc_current_dg >= 0.0 && st_enc_current_dg <= 20.0 && st_enc_last_dg < 360.0 && st_enc_last_dg >= 340.0 ) {
    st_enc_full_rot++;
  }
  if (st_enc_last_dg >= 0.0 && st_enc_last_dg <= 20.0 && st_enc_current_dg < 360.0 && st_enc_current_dg >= 340.0) {
    st_enc_full_rot--;
  }
  stepper_angle_dg = st_enc_current_dg + st_enc_full_rot*360.0;
  st_enc_last_dg  = st_enc_current_dg;

  return;
}

//ControlLoopTask: blinks an LED every 1000 ms
void ControlLoopTask( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  const TickType_t taskPeriod = 20; // ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;){
    // Comms Measured Time: 1.1ms
    //digitalWrite(led_pin, !digitalRead(led_pin)); // Toggle
    digitalWrite(led_pin, HIGH);

    // Arrays for Streaming data
    char step_freq_c[20];
    char out_enc_pulses_c[20];
    char out_sp_rpm_c[20];
    char stepper_angle_dg_c[20];
    char st_enc_current_dg_c[20];
    char stepper_sp_dgps_c[20];

    // Send data
    Serial.print(itoa(step_freq, step_freq_c, 10));
    Serial.print(" ");
    pcnt_get_counter_value(PCNT_UNIT_0, &out_enc_pulses);
    Serial.print(itoa(out_enc_pulses, out_enc_pulses_c, 10));
    Serial.print(" ");//*/
    update_out_sp_rpm();
    dtostrf(out_sp_rpm, 6, 3, out_sp_rpm_c);
    Serial.print(out_sp_rpm_c);
    Serial.print(" ");//*/
    /*stepper_enc_raw = stepper_enc.getRawRotation();
    Serial.println(itoa(stepper_enc_raw, stepper_angle_dg_c, 10));*/
    update_stepper_angle_dg();
    dtostrf(stepper_angle_dg, 6, 3, stepper_angle_dg_c);
    Serial.print(stepper_angle_dg_c);
    Serial.print(" ");//*/
    /*dtostrf(st_enc_current_dg, 6, 3, st_enc_current_dg_c);
    Serial.println(st_enc_current_dg_c);//*/
    // Update stepper speed
    stepper_sp_dgps = (stepper_angle_dg - stepper_last_angle_dg)*50;
    dtostrf(stepper_sp_dgps, 6, 3, stepper_sp_dgps_c);
    Serial.println(stepper_sp_dgps_c);
    stepper_last_angle_dg = stepper_angle_dg;

    // End of communication
    digitalWrite(led_pin, LOW);

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  } 
}

// Custom ISR Functions

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

  //create a task that will be executed in the ControlLoopTask() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    ControlLoopTask,   /* Task function. */
    "ControlLoopTask",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */                  
  delay(500); 

  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);

  ESP32Serial1.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(EN_PIN, OUTPUT);
  ledcSetup(ledChannel, 0, 8);
  ledcAttachPin(STEP_PIN, ledChannel);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware
  digitalWrite(DIR_PIN, LOW);

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

  // Initialize AS5048
  stepper_enc.beginCustom(5000000, 5);
  stepper_enc.setZeroPosition(0);//*/

  // Configure driver
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

  // Debugging AS5048
  //debugAS5048A();

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
    delay(100);
  }
  ledcWrite(ledChannel, 0);

  // Debugging AS5048
  //debugAS5048A();

  // Stop for 4 seconds
  for (int i = 0; i < 50; i++){
    step_freq = 0;
    delay(80);
  }
  
  digitalWrite(DIR_PIN, !digitalRead(DIR_PIN));
  //stepper_dir = !stepper_dir;
  //driver.shaft(stepper_dir);

}

