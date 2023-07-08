/*
Arduino Espressiff version: 2.0.6
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include "driver/pcnt.h"
#include "driver/mcpwm.h"
#include <AS5048A.h>

int led_pin = 19;

HardwareSerial ESP32Serial1(1);
#define DIR_PIN 2                              // Direction
#define STEP_PIN 4                             // Step
#define EN_PIN 5                               // Enable
#define R_SENSE 0.11f                          // SilentStepStick series use 0.11
TMC2208Stepper driver(&ESP32Serial1, R_SENSE); // Hardware Serial
bool stepper_dir = false;                      // CCW, Positive

// setting PWM properties
int16_t step_freq = 0; // 2kHz
const int ledChannel = 0;

// PCNT unit parameters
int out_enc_A = 35;
int out_enc_B = 34;
int16_t out_enc_pulses = 0;
float out_angle_dg = 0.0;

// MCPWM Capture
uint32_t cap_tick_0 = 0;
uint32_t cap_tick_1 = 0;
uint32_t cap_n_ticks = 0;
float out_sp_rpm = 0;
float out_sp_dgps = 0;
bool out_sp_dir = false; // CCW, Positive
static uint8_t out_enc_st_code = 0;
static uint16_t out_enc_st_store = 0;
static int8_t out_enc_st_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};
// Output Encoder Speed Filter
// Butterworth 2nd order - Direct Form I - Single Section - fs=200Hz - fc=8Hz A BIT FASTER, LESS NOISY; NEW BEST ONE
const float out_sp_fil_num[] = {0.01335920, 0.02671840, 0.01335920, 0};
const float out_sp_fil_den[] = {1,-1.64745998, 0.70089678, 0};//*/
/*// Butterworth 2nd order - Direct Form I - Single Section - fs=200Hz - fc=5Hz DELAY 
const float out_sp_fil_num[] = {0.00554272, 0.01108543, 0.00554272, 0};
const float out_sp_fil_den[] = {1,-1.77863177, 0.80080265, 0};//*/
float out_sp_unfiltered[] = {0, 0, 0, 0};
float out_sp_filtered[] = {0, 0, 0, 0};

// SPI Pins
int8_t VSPI_MISO = 26;
int8_t VSPI_SCLK = 25;
int8_t VSPI_SS = 33;
int8_t VSPI_MOSI = 32;
AS5048A stepper_enc(VSPI, VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS, false);
float stepper_angle_dg = 0;
float stepper_angle_offset_dg = 0;
float stepper_sp_dgps = 0;
float stepper_last_angle_dg = 0;
float st_enc_current_dg = 0;
float st_enc_last_dg = 0;
int16_t st_enc_full_rot = 0;
int16_t stepper_enc_raw = 0;
// Stepper Encoder Speed Filter
// Butterworth 2nd order - Direct Form I - Single Section - fs=200Hz - fc=8Hz 
const float st_sp_fil_num[] = {0.01335920, 0.02671840, 0.01335920, 0};
const float st_sp_fil_den[] = {1,-1.64745998, 0.70089678, 0};//*/
float st_sp_unfiltered[] = {0, 0, 0, 0};
float st_sp_filtered[] = {0, 0, 0, 0};

// Multicore
TaskHandle_t ControlLoopTaskHandle;

// Mode selector
// - Open Loop: 0
// - Debug Output Speed Filter: 1
// - Debug Stepper Speed Filter: 2
// - PID mode: 4
int mode_selector = 0;

// Commands (ASCII - String)
// - 49 ('1'): Open Loop - Extenal signal
// - 50 ('2'): Open Loop - Linear Ramp
// - 51 ('3'): Open Loop - Sinusoidal
// - 52 ('4'): PID - Step values
// - 53 ('5'): PID - Linear ramp
// - 54 ('6'): PID - Sinusoidal
int command_msg = 0;

// PID 
//int pid_en_flag = 0;
float pid_set_point = 0;
float pid_err[] = {0, 0, 0};
float pid_u[] = {0, 0, 0};
/*// Output Speed Control
float pid_num[] = {12.19, -11.2, 2.57}; // Works Ok
//float pid_num[] = {100.8, -160, 63.52};// Faster, without noticeable overshoot
float pid_den[] = {1, 0, -1};//*/
/*// Stepper Speed Control
float pid_num[] = {0.3951, -0.3282, 0};
float pid_den[] = {1, -1, 0};//*/
// Output Angle Control
//float pid_num[] = {1692, -1649, 0};// Good One
float pid_num[] = {3290, -3163, 0};// Faster One
float pid_den[] = {1, -1, 0};//*/

// Custom Functions
void set_stepper_angle_offset_dg(){
  stepper_angle_offset_dg = stepper_enc.getRotationInDegrees();
  return;
}

void update_out_angle_dg(){
  pcnt_get_counter_value(PCNT_UNIT_0, &out_enc_pulses);
  out_angle_dg = out_enc_pulses*0.03;
  return;
}

void update_out_sp_rpm(){
  // Calculate
  if (cap_n_ticks == 0) { out_sp_dgps = 0; }
  else { out_sp_dgps = 400000.0/cap_n_ticks; } // Max: 5 RPM

  // Approximate to Zero, Threshold: 0.25 RPM
  if(out_sp_dgps < 0.25) { out_sp_dgps = 0.0; }
  else if (out_sp_dir) { out_sp_dgps = -1*out_sp_dgps; }

  return;
}

void update_out_sp_dgps(){
  // Calculate
  if (cap_n_ticks == 0) { out_sp_dgps = 0; }
  else { out_sp_dgps = 2400000.0/cap_n_ticks; } // Max: 30 dgps

  // Approximate to Zero, Threshold: 0.5 deg/s
  if(out_sp_dgps < 0.5) { out_sp_dgps = 0.0; }
  else if (out_sp_dir) { out_sp_dgps = -1*out_sp_dgps; } 

  // Filter Speed
  out_sp_unfiltered[0] = out_sp_dgps;
  // Second Order
  out_sp_filtered[0] = out_sp_unfiltered[0]*out_sp_fil_num[0] + out_sp_unfiltered[1]*out_sp_fil_num[1] + out_sp_unfiltered[2]*out_sp_fil_num[2];
  out_sp_filtered[0] = out_sp_filtered[0] - out_sp_filtered[1]*out_sp_fil_den[1] - out_sp_filtered[2]*out_sp_fil_den[2];//*/

  // Debug Output Speed Filter  
  if (mode_selector == 1){
    char out_sp_unfiltered_n_c[20]; 
    char out_sp_filtered_n_c[20];
    dtostrf(out_sp_unfiltered[0], 6, 3, out_sp_unfiltered_n_c);
    dtostrf(out_sp_filtered[0], 6, 3, out_sp_filtered_n_c);
    Serial.print(out_sp_unfiltered_n_c);
    Serial.print(" ");
    Serial.println(out_sp_filtered_n_c);//*/
  }  

  // Update Vectors
  out_sp_unfiltered[1] = out_sp_unfiltered[0];
  out_sp_unfiltered[2] = out_sp_unfiltered[1];
  out_sp_filtered[1] = out_sp_filtered[0];
  out_sp_filtered[2] = out_sp_filtered[1];

  return;
}

void update_stepper_angle_dg(){
  st_enc_current_dg = stepper_enc.getRotationInDegrees();

  if (st_enc_current_dg >= 0.0 && st_enc_current_dg <= 25.0 && st_enc_last_dg <= 360.0 && st_enc_last_dg >= 345.0 ) {
    st_enc_full_rot++;
  }
  if (st_enc_last_dg >= 0.0 && st_enc_last_dg <= 25.0 && st_enc_current_dg <= 360.0 && st_enc_current_dg >= 345.0) {
    st_enc_full_rot--;
  }
  stepper_angle_dg = st_enc_current_dg + st_enc_full_rot*360.0 - stepper_angle_offset_dg;
  st_enc_last_dg  = st_enc_current_dg;

  return;
}

void update_stepper_sp_dgps(float fs_Hz){
  stepper_sp_dgps = (stepper_angle_dg - stepper_last_angle_dg)*fs_Hz; // multiplied by fs <> divided by Ts
  if (abs(stepper_sp_dgps) <= 20) stepper_sp_dgps = 0;

  // Filter
  st_sp_unfiltered[0] = stepper_sp_dgps;  
  // Second Order
  st_sp_filtered[0] = st_sp_unfiltered[0]*st_sp_fil_num[0] + st_sp_unfiltered[1]*st_sp_fil_num[1] + st_sp_unfiltered[2]*st_sp_fil_num[2];
  st_sp_filtered[0] = st_sp_filtered[0] - st_sp_filtered[1]*st_sp_fil_den[1] - st_sp_filtered[2]*st_sp_fil_den[2];//*/

  // Debug Stepper Speed Filter
  if (mode_selector == 2){
    char st_sp_unfiltered_n_c[20]; 
    char st_sp_filtered_n_c[20];
    dtostrf(st_sp_unfiltered[0], 6, 3, st_sp_unfiltered_n_c);
    dtostrf(st_sp_filtered[0], 6, 3, st_sp_filtered_n_c);
    Serial.print(st_sp_unfiltered_n_c);
    Serial.print(" ");
    Serial.println(st_sp_filtered_n_c);//*/
  }  

  // Update Vectors
  st_sp_unfiltered[1] = st_sp_unfiltered[0];
  st_sp_unfiltered[2] = st_sp_unfiltered[1];
  st_sp_filtered[1] = st_sp_filtered[0];
  st_sp_filtered[2] = st_sp_filtered[1];

  stepper_last_angle_dg = stepper_angle_dg;
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

//ControlLoopTask: blinks an LED every 1000 ms
void ControlLoopTask( void * pvParameters ){
  //Serial.print("Control Loop Task running on core ");
  //Serial.println(xPortGetCoreID());
  const TickType_t taskPeriod = 5; // ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;){
    digitalWrite(led_pin, HIGH);

    // Arrays for Streaming data
    char step_freq_c[20];
    char out_angle_dg_c[20];
    char out_sp_dgps_c[20];
    char stepper_angle_dg_c[20];
    char stepper_sp_dgps_c[20];
    char set_point_c[20];

    // Update sensor data
    update_out_angle_dg();
    update_out_sp_dgps();
    update_stepper_angle_dg();
    update_stepper_sp_dgps(200); // Ts=5ms -> fs = 200Hz   

    // Open loop
    if (mode_selector == 0){
      // Send data as stream of ASCII characters, takes 250us
      Serial.print(itoa(step_freq, step_freq_c, 10));
      Serial.print(" ");
      dtostrf(out_angle_dg, 6, 3, out_angle_dg_c);
      Serial.print(out_angle_dg_c);
      Serial.print(" ");   
      dtostrf(out_sp_filtered[0], 6, 3, out_sp_dgps_c);
      Serial.print(out_sp_dgps_c);
      Serial.print(" ");
      dtostrf(stepper_angle_dg, 6, 3, stepper_angle_dg_c);
      Serial.print(stepper_angle_dg_c);
      Serial.print(" ");
      dtostrf(st_sp_filtered[0], 6, 3, stepper_sp_dgps_c);
      Serial.println(stepper_sp_dgps_c);//*/
    }     

    // PID loop
    if (mode_selector == 4){
      //pid_err[0] = pid_set_point - out_sp_filtered[0];
      //pid_err[0] = pid_set_point - st_sp_filtered[0];
      pid_err[0] = pid_set_point - out_angle_dg;
      pid_u[0] = pid_err[0]*pid_num[0] + pid_err[1]*pid_num[1] + pid_err[2]*pid_num[2];
      pid_u[0] = pid_u[0] - pid_u[1]*pid_den[1] - pid_u[2]*pid_den[2];

      // Limit acceleration (20 - no load)
      if (abs(pid_u[0]) > step_freq + 5){
        int pid_u_sign;
        if (pid_u[0] >= 0){
          pid_u_sign = 1;
        } else {
          pid_u_sign = -1;
        }
        pid_u[0] = pid_u_sign*(step_freq + 5);
      }

      // Control signal saturation (2800 - no load)
      if (pid_u[0] > 1500){
        pid_u[0] = 1500;
      } else if (pid_u[0] < -1500){
        pid_u[0] = -1500;
      }

      // Set control signal
      step_freq = int(pid_u[0]);
      if (step_freq >= 0){
        stepper_dir = false;
      } else {
        stepper_dir = true;
        step_freq = step_freq*-1;
      }
      driver.shaft(stepper_dir);
      ledcWriteTone(ledChannel, step_freq);

      // Update data
      pid_err[1] = pid_err[0];
      pid_err[2] = pid_err[1];
      pid_u[1] = pid_u[0];
      pid_u[2] = pid_u[1];//*/

      // Send data as stream of ASCII characters, takes 250us
      Serial.print(itoa(step_freq, step_freq_c, 10));
      Serial.print(" ");
      dtostrf(out_angle_dg, 6, 3, out_angle_dg_c);
      Serial.print(out_angle_dg_c);
      Serial.print(" ");
      //dtostrf(out_sp_dgps, 6, 3, out_sp_dgps_c);    
      dtostrf(out_sp_filtered[0], 6, 3, out_sp_dgps_c);
      Serial.print(out_sp_dgps_c);
      Serial.print(" ");
      dtostrf(stepper_angle_dg, 6, 3, stepper_angle_dg_c);
      Serial.print(stepper_angle_dg_c);
      Serial.print(" ");
      dtostrf(st_sp_filtered[0], 6, 3, stepper_sp_dgps_c);
      Serial.print(stepper_sp_dgps_c);
      Serial.print(" ");
      dtostrf(pid_set_point, 6, 3, set_point_c);
      Serial.println(set_point_c);//*/
    }

    // End of loop
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
  // TMC2208 pins
  pinMode(DIR_PIN, OUTPUT);
  ledcSetup(ledChannel, 10, 8);
  ledcAttachPin(STEP_PIN, ledChannel);
  ledcWriteTone(ledChannel, 0);
  pinMode(EN_PIN, OUTPUT);
  ESP32Serial1.begin(115200, SERIAL_8N1, 16, 17);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware
  digitalWrite(DIR_PIN, LOW); // Positive direction

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
  set_stepper_angle_offset_dg();
  //stepper_enc.setZeroPosition(0);

  // Configure driver
  driver.begin();           // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);           // Enables driver in software
  driver.rms_current(1500); // Set motor RMS current
  driver.microsteps(4);     // Set microsteps to 1/4th
  driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);   // Needed for stealthChop
  driver.shaft(stepper_dir);

  // Utilities
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);

  delay(100);

  // Control Loop Task on core 0
  xTaskCreatePinnedToCore(
    ControlLoopTask,   /* Task function. */
    "ControlLoopTask",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &ControlLoopTaskHandle,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */ 

  delay(500); 
}

void loop(){
  if (Serial.available() >0){
    command_msg = Serial.read();
  }

  // Open Loop - External signal (49 - '1')
  if (command_msg == 49){
    mode_selector = 0;
    command_msg = 0;
  }

  // Open Loop - Linear Ramp (50 - '2')
  if (command_msg == 50){
    mode_selector = 0;
    command_msg = 0;
    // Linear Ramp using LedCPWM
    for (int i = 0; i < 100; i++){
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
  }
  
  // Open Loop - Sinusoidal (51 - '3')
  if (command_msg == 51){ 
    mode_selector = 0;
    command_msg = 0;
    // Sinusoidal Ramp using LedCPWM
    float step_freq_f = 0;
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
    ledcWrite(ledChannel, 0);//*/

    // Stop for 4 seconds
    for (int i = 0; i < 50; i++){
      step_freq = 0;
      delay(80);
    }
  }

  // PID - Step values (52 - '4')
  if (command_msg == 52){
    // Set point values:
    // - outputSpeed: 30 dg/s
    // - stepperSpeed: 900 dg/s 
    // - outputAngle: 180 dg
    mode_selector = 4;
    command_msg = 0;

    // Step values
    pid_set_point = 45;
    delay(8000);
    pid_set_point = -15;
    delay(6000);
    pid_set_point = 15;
    delay(7000);
    pid_set_point = -45;
    delay(6000);
    pid_set_point = 0;//*/
  }

  // PID - Linear Ramp (53 - '5')
  if (command_msg == 53){
    // Set point values:
    // - outputSpeed: 30 dg/s
    // - stepperSpeed: 900 dg/s 
    // - outputAngle: 180 dg
    mode_selector = 4;
    command_msg = 0;

    // Linear Ramp
    float max_set_point = 240;
    pid_set_point = 0;
    for (int i = 0; i < 100; i++){
      if (i < 25){
        pid_set_point = max_set_point*i/25.0;
      } else if (i < 75){
        pid_set_point = max_set_point;
      } else {
        pid_set_point = max_set_point*(100-i)/25.0;
      }
      delay(100);
    }
    pid_set_point = 0;//*/
  }  

  // PID - Sinusoidal (54 - '6')
  if (command_msg == 54){ // ASCII for 6
    // Set point values:
    // - outputSpeed: 30 dg/s
    // - stepperSpeed: 900 dg/s 
    // - outputAngle: 180 dg
    mode_selector = 4;
    command_msg = 0;    

    // Sinusoidal signal
    // - sin(k*i + fi) -> k = 2*pi/10*T (10 - delay(100))
    float sin_angle = 0;
    for (int i = 0; i < 150; i++){
      sin_angle = 0.1047 * i - 1.570796; // 6s - 0.1047
      pid_set_point = 30 * sin(sin_angle) + 30;
      /*if (i < 30){
        sin_angle = 0.104719333 * i - 1.570796;
        pid_set_point = 30 * sin(sin_angle) + 30;
      } else if (i < 70){
        pid_set_point = 60;
      } else {
        sin_angle = 0.104719333 * i + 0.5235;
        pid_set_point = 30 * sin(sin_angle) + 30;
      }*/
      delay(100);
    }
    pid_set_point = 0;//*/
  }

}

