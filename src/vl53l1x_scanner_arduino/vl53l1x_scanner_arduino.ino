#include <Wire.h>
#include "vl53l1_api.h"
#define __THIS_IS_THE_SCANNER__
#include "shared_conf.h"

//-------------------- GLOBAL VARIABLES ---------------------------------------
VL53L1_Dev_t sensor;
VL53L1_UserRoi_t roiConfig;
VL53L1_RangingMeasurementData_t rangingData;
int stepper_pos = 0; // managed by oneStep and calibration section
bool is_active = false;
bool is_initialized = false;
bool is_first_scan_received = false;
char* input_string;
int input_size = 0;
int scanner_direction = 1;
int desired_step = 0;

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
    if (p_scanner_mode == SCAN_MODE_CAM_DEPTHIMAGE)
    {
      Serial.println(F("$")); // FULL SCAN HEADER
    }
    // REWIND
    else if (p_scanner_rewind && stepper_pos >= p_stepper_step_max)
    { 
      Serial.println(F("$+")); // FULL SCAN HEADER
      is_first_scan_received = false;
      stepperStep(-stepper_pos+p_stepper_step_min, p_stepper_delay);
    } 
    // CHANGE DIRECTION
    else if ( !p_scanner_rewind && ((stepper_pos >= p_stepper_step_max && scanner_direction > 0) || (stepper_pos <= p_stepper_step_min && scanner_direction < 0)) )
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
      // default:
      desired_step = scanner_direction*p_scanner_horizontal_steps;

      // adaptive resolution (only compatible with SCAN_MODE_2D_POINTCLOUD)
      if (p_adaptive_resolution_enable && is_first_scan_received && p_scanner_mode == SCAN_MODE_2D_POINTCLOUD)
      {
        float distance = (float(rangingData.RangeMilliMeter)/1000.0);
        float adaptive_angle = (1.0/distance)*p_adaptive_resolution_coef_0 + p_adaptive_resolution_coef_1;
        float step_per_angle = 1.0 / p_stepper_horizontal_angle_per_step;
        float target_step = adaptive_angle * step_per_angle;
        if (abs(int(target_step)) <= 0)
          target_step = 1;
        desired_step = abs(int(target_step)) * scanner_direction;
      }

      //Serial.print("# desired_step ");
      //Serial.println(desired_step);
      stepperStep(desired_step, p_stepper_delay);
    }
}


void scanState()
{
  switch (p_scanner_mode)
  {
    case SCAN_MODE_2D_LASERSCAN: scan2D(); break;
    case SCAN_MODE_2D_POINTCLOUD: scan2D(); break;
    case SCAN_MODE_3D_POINTCLOUD: scan3D(); break;
    case SCAN_MODE_CAM_DEPTHIMAGE: scanCamera(); break;
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
  for (int i=0; i<13; i+=p_scanner_vertical_steps)
  {
    measureDistance(stepper_pos, i+LASER_VERTICAL_POS_BIAS, true, p_laser_roi_topleftx, i+3, p_laser_roi_botrightx, i);
  }
}

void scanCamera()
{
  for (int i=12; i>=0; i-=p_scanner_vertical_steps)
  {
    for (int j=12; j>=0; j-=p_scanner_horizontal_steps)
    {
      measureDistance(j+LASER_HORIZONTAL_POS_BIAS, i+LASER_VERTICAL_POS_BIAS, true, j, i+3, j+3, i); // hori, vert
    }
    
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
      // V -> set variables
      // I -> initialize
      // C -> calibration
      // S -> start
      // P -> poweroff
      // Serial.println(input_string); // debug
      char* token = strtok(input_string, " ");

      if (token[0] == 'V') // I was unable to use switch statement here. It didn't work and I don't know why
      {
        static bool first_time = true;
        if (first_time)
        {
          Serial.println(F("# ===== SET VARIABLE COMMAND ====="));
          first_time = false;
        }
        token = strtok(NULL, " ");
        PROCESS_PARAMETER(atoi(token));
      }
      else if (token[0] == 'I')  
      {
        Serial.println(F("# ===== INITIALIZE COMMAND ====="));
        initialize_hardware();
      }
      else if (token[0] == 'C')
      {
        Serial.println(F("# ===== CALIBRATION COMMAND ====="));
        token = strtok(NULL, " ");
        int calib_command = atoi(token);
        if (calib_command > p_scanner_calibration_max_value) calib_command = p_scanner_calibration_max_value;
        if (calib_command < -p_scanner_calibration_max_value) calib_command = -p_scanner_calibration_max_value;
        stepperStep(calib_command, p_stepper_delay);
        stepper_pos = 0;
      }
      else if (token[0] == 'S')
      {
        Serial.println(F("# ===== START COMMAND ====="));
        if (p_scanner_mode != SCAN_MODE_CAM_DEPTHIMAGE)
          stepperStep(p_stepper_step_min, p_stepper_delay); // bring stepper to pos 0
        is_active = true;
        Serial.println("$+"); // FULL SCAN HEADER
      }
      else if (token[0] == 'P')
      {
        Serial.println(F("# ===== SHUTDOWN COMMAND ====="));
        is_active = false;
        stepperStep(-stepper_pos, p_stepper_delay); // bring stepper to pos 0
        digitalWrite(p_stepper_pin_1, LOW);
        digitalWrite(p_stepper_pin_2, LOW);
        digitalWrite(p_stepper_pin_3, LOW);
        digitalWrite(p_stepper_pin_4, LOW);
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
  Serial.println(F("#   Initializing Stepper"));
  delay(100);
  pinMode(p_stepper_pin_1, OUTPUT);
  pinMode(p_stepper_pin_2, OUTPUT);
  pinMode(p_stepper_pin_3, OUTPUT);
  pinMode(p_stepper_pin_4, OUTPUT);

  // Initialize VL53L1X
  Serial.println(F("#   Initializing VL53L1X"));
  delay(100);
  Wire.begin();
  Wire.setClock(LASER_WIRE_CLOCK);
  sensor.I2cDevAddr   = LASER_DEVICE_ADDR;
  roiConfig.TopLeftX  = p_laser_roi_topleftx;
  roiConfig.TopLeftY  = p_laser_roi_toplefty; //9
  roiConfig.BotRightX = p_laser_roi_botrightx;
  roiConfig.BotRightY = p_laser_roi_botrighty; // 6
  check( VL53L1_software_reset(&sensor) );
  check( VL53L1_WaitDeviceBooted(&sensor) );
  check( VL53L1_DataInit(&sensor) );
  check( VL53L1_StaticInit(&sensor) );
  check( VL53L1_SetDistanceMode(&sensor, p_laser_distance_mode) ); // VL53L1_DISTANCEMODE_LONG
  check( VL53L1_SetMeasurementTimingBudgetMicroSeconds(&sensor, p_laser_measurement_timing_budget_micro_seconds) ); // 50000
  check( VL53L1_SetInterMeasurementPeriodMilliSeconds(&sensor, p_laser_inter_measurement_period_milli_seconds) ); // 50
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

  is_first_scan_received = true;
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
      digitalWrite(p_stepper_pin_1, LOW);
      digitalWrite(p_stepper_pin_2, LOW);
      digitalWrite(p_stepper_pin_3, STEPPER_TWO_PHASE);
      digitalWrite(p_stepper_pin_4, HIGH);
      break;
      case 1:
      digitalWrite(p_stepper_pin_1, LOW);
      digitalWrite(p_stepper_pin_2, STEPPER_TWO_PHASE);
      digitalWrite(p_stepper_pin_3, HIGH);
      digitalWrite(p_stepper_pin_4, LOW);
      break;
      case 2:
      digitalWrite(p_stepper_pin_1, STEPPER_TWO_PHASE);
      digitalWrite(p_stepper_pin_2, HIGH);
      digitalWrite(p_stepper_pin_3, LOW);
      digitalWrite(p_stepper_pin_4, LOW);
      break;
      case 3:
      digitalWrite(p_stepper_pin_1, HIGH);
      digitalWrite(p_stepper_pin_2, LOW);
      digitalWrite(p_stepper_pin_3, LOW);
      digitalWrite(p_stepper_pin_4, STEPPER_TWO_PHASE);
    } 
  }
  else
  {
    stepper_pos--;
    switch(step_number)
    {
      case 0:
      digitalWrite(p_stepper_pin_1, HIGH);
      digitalWrite(p_stepper_pin_2, LOW);
      digitalWrite(p_stepper_pin_3, LOW);
      digitalWrite(p_stepper_pin_4, STEPPER_TWO_PHASE);
      break;
      case 1:
      digitalWrite(p_stepper_pin_1, STEPPER_TWO_PHASE);
      digitalWrite(p_stepper_pin_2, HIGH);
      digitalWrite(p_stepper_pin_3, LOW);
      digitalWrite(p_stepper_pin_4, LOW);
      break;
      case 2:
      digitalWrite(p_stepper_pin_1, LOW);
      digitalWrite(p_stepper_pin_2, STEPPER_TWO_PHASE);
      digitalWrite(p_stepper_pin_3, HIGH);
      digitalWrite(p_stepper_pin_4, LOW);
      break;
      case 3:
      digitalWrite(p_stepper_pin_1, LOW);
      digitalWrite(p_stepper_pin_2, LOW);
      digitalWrite(p_stepper_pin_3, STEPPER_TWO_PHASE);
      digitalWrite(p_stepper_pin_4, HIGH);
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
