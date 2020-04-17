
#include <Wire.h>
#include "vl53l1_api.h"

//-----------------------------------------------------------------------------
#define MIN(a,b) ((a)<(b)?(a):(b))
#define INITIALIZE_PARAMETER(a) {if( (token = strtok(NULL, " ")) != NULL ) a = atol(token); Serial.print(F("#   ")); Serial.print(F(#a)); Serial.print(F(": ")); Serial.println((a));}

//-------------------- PROTOTYPES ---------------------------------------------
void initialize();
void scan2D();
void scan3D();
void scan();
int check(int err);
void stepperStep(int movement, unsigned long delay_time);
void oneStep(bool dir, unsigned long delay_time);

//-------------------- GLOBAL VARIABLES ---------------------------------------
VL53L1_Dev_t sensor;
VL53L1_UserRoi_t roiConfig;
VL53L1_RangingMeasurementData_t rangingData;
int stepper_pos = 0; // managed by oneStep and calibration section
bool is_active = false;
bool is_initialized = false;
char* input_string;
int input_size = 0;
int scanner_direction = 1;

//------------------- PARAMETERS (in order) ----------------------------------
int stepper_pins[4] = {9,10,11,12};
int stepper_delay = 2250; // my stepper motor misses step with 2000. So I added extra 250
int stepper_step_min = -512;
int stepper_step_max = +512;
int laser_distance_mode = 3; // ADAPTIVE(0), short(1), medium(2), long(3)
long laser_measurement_timing_budget_micro_seconds = 10000;
int laser_inter_measurement_period_milli_seconds = 1;
int scanner_3d_mode = 0;
int scanner_3d_vertical_steps_per_scan = 1;
int scanner_horizontal_steps_per_scan = 32;
int scanner_rewind = 0;
int scanner_calibration_max_value = 500;

//-------------- CONTANT PARAMETERS -------------------------------------------
const int input_string_max_size = 96;

//-----------------------------------------------------------------------------
void setup()
{
  // Setup arduino
  input_string = (char*)malloc(input_string_max_size*sizeof(char)); // I was unable to decleare it in stack memory. I don't know why
  input_string[input_string_max_size-1] = '\0';
	Serial.begin(9600);
  while (!Serial) {;} // wait for serial port to connect.
}


void loop()
{
  if (is_active && is_initialized)
  {
    //scan();

    // REWIND
    if (scanner_rewind && stepper_pos >= stepper_step_max)
    { 
      Serial.println("$+"); // FULL SCAN HEADER
      stepperStep(-stepper_pos+stepper_step_min, stepper_delay);
    } 
    // CHANGE DIRECTION
    else if ( !scanner_rewind && ((stepper_pos >= stepper_step_max && scanner_direction > 0) || (stepper_pos <= stepper_step_min && scanner_direction < 0)) )
    {
      Serial.print("$"); // FULL SCAN HEADER
      scanner_direction = -scanner_direction;
      if (scanner_direction>0)
        Serial.println("+");
      else
        Serial.println("-");
    }
    else
    {
      stepperStep(scanner_direction*scanner_horizontal_steps_per_scan, stepper_delay);
      scan();
    }
  }

}


void serialEvent()
{
  while (Serial.available())
  {
    char in_char = (char)Serial.read();
    if (in_char != '\n')
    {
      input_string[input_size] = in_char;
      input_size++;
    }
    else
    {
      input_string[input_size] = '\0';
      ////////// PROCESS INPUT HERE //////////
      // How to use this device: I -> C (optional) -> S -> P
      // I -> initialize
      // C -> calibration
      // S -> start
      // P -> poweroff
      // Serial.println(input_string); // For debugging purposes
      char* token = strtok(input_string, " ");
      
      if (token[0] == 'I')  // I was unable to use switch statement here. It didn't work and I don't know why
      {
        Serial.println(F("# ===== INITIALIZE ====="));
        Serial.println(F("# [ROS Parameters]"));
        // Stepper Motor Parameters
        INITIALIZE_PARAMETER(stepper_pins[0]);
        INITIALIZE_PARAMETER(stepper_pins[1]);
        INITIALIZE_PARAMETER(stepper_pins[2]);
        INITIALIZE_PARAMETER(stepper_pins[3]);
        INITIALIZE_PARAMETER(stepper_delay);
        INITIALIZE_PARAMETER(stepper_step_min);
        INITIALIZE_PARAMETER(stepper_step_max);
        // Laser Parameters
        INITIALIZE_PARAMETER(laser_distance_mode);
        INITIALIZE_PARAMETER(laser_measurement_timing_budget_micro_seconds);
        INITIALIZE_PARAMETER(laser_inter_measurement_period_milli_seconds);
        // Scanner Parameters
        INITIALIZE_PARAMETER(scanner_3d_mode);
        INITIALIZE_PARAMETER(scanner_3d_vertical_steps_per_scan);
        INITIALIZE_PARAMETER(scanner_horizontal_steps_per_scan);
        INITIALIZE_PARAMETER(scanner_rewind);
        INITIALIZE_PARAMETER(scanner_calibration_max_value);
        
        initialize();
      }
      else if (token[0] == 'C')
      {
        Serial.println(F("# ===== CALIBRATION ====="));
        token = strtok(NULL, " ");
        int calib_command = atoi(token);
        if (calib_command > scanner_calibration_max_value) calib_command = scanner_calibration_max_value;
        if (calib_command < -scanner_calibration_max_value) calib_command = -scanner_calibration_max_value;
        stepperStep(calib_command, stepper_delay);
        stepper_pos = 0;
      }
      else if (token[0] == 'S')
      {
        Serial.println(F("# ===== START COMMAND ====="));
        stepperStep(stepper_step_min, stepper_delay); // bring stepper to pos 0
        is_active = true;
        Serial.println("$+"); // FULL SCAN HEADER
      }
      else if (token[0] == 'P')
      {
        Serial.println(F("# ===== SHUTDOWN ====="));
        is_active = false;
        stepperStep(-stepper_pos, stepper_delay); // bring stepper to pos 0
        digitalWrite(stepper_pins[0], LOW);
        digitalWrite(stepper_pins[1], LOW);
        digitalWrite(stepper_pins[2], LOW);
        digitalWrite(stepper_pins[3], LOW);
      }
      
      ////////////////////////////////////////
      
      input_string[0] = '\0';
      input_size = 0;
    } 
  }
}


void initialize()
{
  Serial.println(F("# [Init Hardware]"));
  // Initialize Stepper
  Serial.println(F("#   Initializing Stepper..."));
  pinMode(stepper_pins[0], OUTPUT);
  pinMode(stepper_pins[1], OUTPUT);
  pinMode(stepper_pins[2], OUTPUT);
  pinMode(stepper_pins[3], OUTPUT);

  // Initialize VL53L1X
  Serial.println(F("#   Initializing VL53L1X..."));
  Wire.begin();
  Wire.setClock(400000);
  sensor.I2cDevAddr   = 0x52;
  // smallest and centered possible roi (4x4): TODO
  roiConfig.TopLeftX  = 6;
  roiConfig.TopLeftY  = 15; //9
  roiConfig.BotRightX = 9;
  roiConfig.BotRightY = 0; // 6
  check( VL53L1_software_reset(&sensor) );
  check( VL53L1_WaitDeviceBooted(&sensor) );
  check( VL53L1_DataInit(&sensor) );
  check( VL53L1_StaticInit(&sensor) );
  check( VL53L1_SetDistanceMode(&sensor, laser_distance_mode) ); // VL53L1_DISTANCEMODE_LONG
  check( VL53L1_SetMeasurementTimingBudgetMicroSeconds(&sensor, laser_measurement_timing_budget_micro_seconds) ); // 50000
  check( VL53L1_SetInterMeasurementPeriodMilliSeconds(&sensor, laser_inter_measurement_period_milli_seconds) ); // 50
  check( VL53L1_SetUserROI(&sensor, &roiConfig) );
  check( VL53L1_StartMeasurement(&sensor) );

  Serial.println(F("# [VL53L1X Information]"));

  // device version/model information
  uint8_t byteData;
  check( VL53L1_RdByte(&sensor, 0x010F, &byteData) );
  Serial.print(F("#   Model ID: "));
  Serial.println(byteData, HEX);
  check( VL53L1_RdByte(&sensor, 0x0110, &byteData) );
  Serial.print(F("#   Module Type: "));
  Serial.println(byteData, HEX);

  uint8_t product_rev_major, product_rev_minor;
  check( VL53L1_GetProductRevision(&sensor, &product_rev_major, &product_rev_minor) );
  Serial.print(F("#   Product Revision: "));
  Serial.print(product_rev_major);
  Serial.print(F("."));
  Serial.println(product_rev_minor);

  VL53L1_Version_t version_inf;
  check( VL53L1_GetVersion(&version_inf) );
  Serial.print(F("#   API Version: "));
  Serial.print(version_inf.major);
  Serial.print(F("."));
  Serial.print(version_inf.minor);
  Serial.print(F(" build "));
  Serial.print(version_inf.build);
  Serial.print(F(" revision "));
  Serial.println(version_inf.revision);
/*
  long unsigned int optical_center_x, optical_center_y;
  check( VL53L1_GetOpticalCenter(&sensor, &optical_center_x, &optical_center_y) );
  Serial.print(F("#   Optical Center X: "));
  Serial.println(optical_center_x);
  Serial.print(F("#   Optical Center Y: "));
  Serial.println(optical_center_y);
*/
  is_initialized = true;
}

void scan2D()
{
  VL53L1_clear_interrupt_and_enable_next_range(&sensor, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT);
  check( VL53L1_WaitMeasurementDataReady(&sensor) );
  check(VL53L1_GetRangingMeasurementData(&sensor, &rangingData));
  Serial.print(stepper_pos); // horizontal pos
  Serial.print(F(" "));
  Serial.print(0);   // vertical pos
  Serial.print(F(" "));
  Serial.print(rangingData.RangeMilliMeter);
  Serial.print(F(" "));
  Serial.print(rangingData.RangeStatus);
  Serial.print(F(" "));
  Serial.print(rangingData.SignalRateRtnMegaCps/65536.0);
  Serial.print(F(" "));
  Serial.println(rangingData.AmbientRateRtnMegaCps/65336.0);
}

void scan3D()
{
  for (int i=0; i<13; i+=scanner_3d_vertical_steps_per_scan)
  {
    roiConfig.TopLeftY=i+3;
    roiConfig.BotRightY=i;
    check( VL53L1_SetUserROI(&sensor, &roiConfig) );
    
    VL53L1_clear_interrupt_and_enable_next_range(&sensor, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT);
    check( VL53L1_WaitMeasurementDataReady(&sensor) );
    check(VL53L1_GetRangingMeasurementData(&sensor, &rangingData));
    Serial.print(stepper_pos); // horizontal pos
    Serial.print(F(" "));
    Serial.print(i-6);           // vertical pos (-6 is constant for 13 vertical scans)
    Serial.print(F(" "));
    Serial.print(rangingData.RangeMilliMeter);
    Serial.print(F(" "));
    Serial.print(rangingData.RangeStatus);
    Serial.print(F(" "));
    Serial.print(rangingData.SignalRateRtnMegaCps/65536.0);
    Serial.print(F(" "));
    Serial.println(rangingData.AmbientRateRtnMegaCps/65336.0);
  }
}

void scan()
{
  if (scanner_3d_mode)
  {
    scan3D();
  }
  else
  {
    scan2D();
  }
}


//=====================================
//=============== UTILITY =============
//=====================================

int check(int err)
{
  if (err != 0)
  {
    Serial.print(F("# ERROR: "));
    Serial.println(err);
  }
  return err;
}


//=====================================
//=============== stepper ===============
//=====================================

void stepperStep(int movement, unsigned long delay_time){
  if (movement >= 0)
    for (int i=0; i<movement; i++)
      oneStep(true, delay_time);
  else
    for (int i=0; i<-movement; i++)
      oneStep(false, delay_time);
}

// full step / one phase
// Because we don't need two phases and also mine stepper motor was missing steps with two phases and/or half-step 
void oneStep(bool dir, unsigned long delay_time){
  static int step_number = 0;
  static unsigned long last_movement_time = 0;
  
  if(dir){
    stepper_pos++;
    switch(step_number)
    {
      case 0:
      digitalWrite(stepper_pins[0], LOW);
      digitalWrite(stepper_pins[1], LOW);
      digitalWrite(stepper_pins[2], LOW);
      digitalWrite(stepper_pins[3], HIGH);
      break;
      case 1:
      digitalWrite(stepper_pins[0], LOW);
      digitalWrite(stepper_pins[1], LOW);
      digitalWrite(stepper_pins[2], HIGH);
      digitalWrite(stepper_pins[3], LOW);
      break;
      case 2:
      digitalWrite(stepper_pins[0], LOW);
      digitalWrite(stepper_pins[1], HIGH);
      digitalWrite(stepper_pins[2], LOW);
      digitalWrite(stepper_pins[3], LOW);
      break;
      case 3:
      digitalWrite(stepper_pins[0], HIGH);
      digitalWrite(stepper_pins[1], LOW);
      digitalWrite(stepper_pins[2], LOW);
      digitalWrite(stepper_pins[3], LOW);
    } 
  }
  else
  {
    stepper_pos--;
    switch(step_number)
    {
      case 0:
      digitalWrite(stepper_pins[0], HIGH);
      digitalWrite(stepper_pins[1], LOW);
      digitalWrite(stepper_pins[2], LOW);
      digitalWrite(stepper_pins[3], LOW);
      break;
      case 1:
      digitalWrite(stepper_pins[0], LOW);
      digitalWrite(stepper_pins[1], HIGH);
      digitalWrite(stepper_pins[2], LOW);
      digitalWrite(stepper_pins[3], LOW);
      break;
      case 2:
      digitalWrite(stepper_pins[0], LOW);
      digitalWrite(stepper_pins[1], LOW);
      digitalWrite(stepper_pins[2], HIGH);
      digitalWrite(stepper_pins[3], LOW);
      break;
      case 3:
      digitalWrite(stepper_pins[0], LOW);
      digitalWrite(stepper_pins[1], LOW);
      digitalWrite(stepper_pins[2], LOW);
      digitalWrite(stepper_pins[3], HIGH);
      break;
    } 
  }
  step_number = (step_number+1) % 4;
  delayMicroseconds(delay_time);
}
