#include <Wire.h>
#include "vl53l1_api.h"

// Arduino constants <-> Driver constants
#define VL53L1X_DEVICE_ADDR               (0x52)
#define VL53L1X_WIRE_CLOCK                (400000)
#define VL53L1X_ROI_MINCENTERED_TOPLEFTX  (6)
#define VL53L1X_ROI_MINCENTERED_TOPLEFTY  (9) // 15
#define VL53L1X_ROI_MINCENTERED_BOTRIGHTX (9)
#define VL53L1X_ROI_MINCENTERED_BOTRIGHTY (6) // 0
#define VERTICAL_POS_BIAS                 (-6)
#define ARDUINO_SERIAL_BAUD_RATE          (9600)
#define ARDUINO_INPUT_STRING_SIZE         (96)
#define SCAN_MODE_2D                      (1)
#define SCAN_MODE_3D                      (2)
#define SCAN_MODE_CAMERA                  (3)

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
int laser_roi_topleftx = VL53L1X_ROI_MINCENTERED_TOPLEFTX;
int laser_roi_toplefty = VL53L1X_ROI_MINCENTERED_TOPLEFTY;
int laser_roi_botrightx = VL53L1X_ROI_MINCENTERED_BOTRIGHTX;
int laser_roi_botrighty = VL53L1X_ROI_MINCENTERED_BOTRIGHTY;
int laser_distance_mode = 3; // short(1), medium(2), long(3)
long laser_measurement_timing_budget_micro_seconds = 10000;
int laser_inter_measurement_period_milli_seconds = 1;
int scanner_mode = 1;
int scanner_3d_vertical_steps_per_scan = 1;
int scanner_horizontal_steps_per_scan = 32;
int scanner_rewind = 0;
int scanner_calibration_max_value = 500;




//==============================================================================
//=== SETUP & LOOP =============================================================
//==============================================================================

void setup()
{
  // Setup arduino
  input_string = (char*)malloc(ARDUINO_INPUT_STRING_SIZE*sizeof(char)); // I was unable to decleare it in stack memory. I don't know why
  if (input_string == NULL)
  {
    Serial.print(F("# FATAL ERROR: CANT ALLOCATE INPUT STRING IN MEMORY"));
  }
  input_string[ARDUINO_INPUT_STRING_SIZE-1] = '\0';
	Serial.begin(ARDUINO_SERIAL_BAUD_RATE);
  while (!Serial) {;} // wait for serial port to connect.
}


void loop()
{
  if (is_active && is_initialized)
  {
    scanState();
    actionState();
  }
}

//==============================================================================
//=== SCANNER STATE ============================================================
//==============================================================================

void actionState()
{
    // REWIND
    if (scanner_rewind && stepper_pos >= stepper_step_max)
    { 
      Serial.println(F("$+")); // FULL SCAN HEADER
      stepperStep(-stepper_pos+stepper_step_min, stepper_delay);
    } 
    // CHANGE DIRECTION
    else if ( !scanner_rewind && ((stepper_pos >= stepper_step_max && scanner_direction > 0) || (stepper_pos <= stepper_step_min && scanner_direction < 0)) )
    {
      Serial.print(F("$")); // FULL SCAN HEADER
      scanner_direction = -scanner_direction;
      if (scanner_direction>0)
        Serial.println(F("+"));
      else
        Serial.println(F("-"));
    }
    else
    {
      stepperStep(scanner_direction*scanner_horizontal_steps_per_scan, stepper_delay);
    }
}


void scanState()
{
  switch (scanner_mode)
  {
    case 1: scan2D(); break;
    case 2: scan3D(); break;
    case 3: /*camera*/ break;
    default: Serial.println(F("# ERROR: Unknown scanner_mode"));
  }
}

//==============================================================================
//=== SCAN MODES ===============================================================
//==============================================================================

void scan2D()
{
  measureDistance(stepper_pos, 0, false, -1, -1, -1, -1);
}

void scan3D()
{
  for (int i=0; i<13; i+=scanner_3d_vertical_steps_per_scan)
  {
    // i+VERTICAL_POS_BIAS
    //roiConfig.TopLeftY=i+3;
    //roiConfig.BotRightY=i;
  }
}


//==============================================================================
//=== COMMUNICATION & INITIALIZTION ============================================
//==============================================================================

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
      // Serial.println(input_string); // debug
      char* token = strtok(input_string, " ");
      
      if (token[0] == 'I')  // I was unable to use switch statement here. It didn't work and I don't know why
      {
        Serial.println(F("# ===== INITIALIZE COMMAND ====="));
        Serial.println(F("# [Read Parameters]"));

#define INITIALIZE_PARAMETER(a) {if( (token = strtok(NULL, " ")) != NULL ) a = atol(token); Serial.print(F("#   ")); Serial.print(F(#a)); Serial.print(F(": ")); Serial.println((a));}
        
        // Stepper Motor Parameters
        /*INITIALIZE_PARAMETER(stepper_pins[0]);
        INITIALIZE_PARAMETER(stepper_pins[1]);
        INITIALIZE_PARAMETER(stepper_pins[2]);
        INITIALIZE_PARAMETER(stepper_pins[3]);
        INITIALIZE_PARAMETER(stepper_delay);
        INITIALIZE_PARAMETER(stepper_step_min);
        INITIALIZE_PARAMETER(stepper_step_max);
        // Laser Parameters
        INITIALIZE_PARAMETER(laser_roi_topleftx);
        INITIALIZE_PARAMETER(laser_roi_toplefty);
        INITIALIZE_PARAMETER(laser_roi_botrightx);
        INITIALIZE_PARAMETER(laser_roi_botrighty);
        INITIALIZE_PARAMETER(laser_distance_mode);
        INITIALIZE_PARAMETER(laser_measurement_timing_budget_micro_seconds);
        INITIALIZE_PARAMETER(laser_inter_measurement_period_milli_seconds);
        // Scanner Parameters
        
        INITIALIZE_PARAMETER(scanner_mode);
        INITIALIZE_PARAMETER(scanner_3d_vertical_steps_per_scan);
        INITIALIZE_PARAMETER(scanner_horizontal_steps_per_scan);
        INITIALIZE_PARAMETER(scanner_rewind);
        INITIALIZE_PARAMETER(scanner_calibration_max_value);
        */
        
        initialize_hardware();
      }
      else if (token[0] == 'C')
      {
        Serial.println(F("# ===== CALIBRATION COMMAND ====="));
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
        Serial.println(F("# ===== SHUTDOWN COMMAND ====="));
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


void initialize_hardware()
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
  Wire.setClock(VL53L1X_WIRE_CLOCK);
  sensor.I2cDevAddr   = VL53L1X_DEVICE_ADDR;
  roiConfig.TopLeftX  = laser_roi_topleftx;
  roiConfig.TopLeftY  = laser_roi_toplefty; //9
  roiConfig.BotRightX = laser_roi_botrightx;
  roiConfig.BotRightY = laser_roi_botrighty; // 6
  check( VL53L1_software_reset(&sensor) );
  check( VL53L1_WaitDeviceBooted(&sensor) );
  check( VL53L1_DataInit(&sensor) );
  check( VL53L1_StaticInit(&sensor) );
  check( VL53L1_SetDistanceMode(&sensor, laser_distance_mode) ); // VL53L1_DISTANCEMODE_LONG
  check( VL53L1_SetMeasurementTimingBudgetMicroSeconds(&sensor, laser_measurement_timing_budget_micro_seconds) ); // 50000
  check( VL53L1_SetInterMeasurementPeriodMilliSeconds(&sensor, laser_inter_measurement_period_milli_seconds) ); // 50
  check( VL53L1_SetUserROI(&sensor, &roiConfig) );
  check( VL53L1_StartMeasurement(&sensor) );

  Serial.println(F("# [Read VL53L1X Information]"));

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

  long unsigned int optical_center_x, optical_center_y;
  check( VL53L1_GetOpticalCenter(&sensor, &optical_center_x, &optical_center_y) );
  Serial.print(F("#   Optical Center X: "));
  Serial.println(optical_center_x);
  Serial.print(F("#   Optical Center Y: "));
  Serial.println(optical_center_y);

  is_initialized = true;
}

//==============================================================================
//=============== RANGE FINDER =================================================
//==============================================================================

void measureDistance(int horizontal_pos, int vertical_pos, bool update_roi, int roi_topleftx, int roi_toplefty, int roi_botrightx, int roi_botrighty)
{
  if (update_roi)
  {
    roiConfig.TopLeftX  = roi_topleftx;
    roiConfig.TopLeftY  = roi_toplefty;
    roiConfig.BotRightX = roi_botrightx;
    roiConfig.BotRightY = roi_botrighty;
  }
  
  check( VL53L1_SetUserROI(&sensor, &roiConfig) );
  check( VL53L1_clear_interrupt_and_enable_next_range(&sensor, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT));
  check( VL53L1_WaitMeasurementDataReady(&sensor) );
  check( VL53L1_GetRangingMeasurementData(&sensor, &rangingData) );
  
  Serial.print(horizontal_pos); // horizontal pos
  Serial.print(F(" "));
  Serial.print(vertical_pos);   // vertical pos
  Serial.print(F(" "));
  Serial.print(rangingData.RangeMilliMeter);
  Serial.print(F(" "));
  Serial.print(rangingData.RangeStatus);
  Serial.print(F(" "));
  Serial.print(rangingData.SignalRateRtnMegaCps/65536.0);
  Serial.print(F(" "));
  Serial.println(rangingData.AmbientRateRtnMegaCps/65336.0);
}

//==============================================================================
//=============== STEPPER ======================================================
//==============================================================================

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


//==============================================================================
//=============== UTILITY ======================================================
//==============================================================================

int check(int err)
{
  if (err != 0)
  {
    Serial.print(F("# ERROR: "));
    Serial.println(err);
  }
  return err;
}
