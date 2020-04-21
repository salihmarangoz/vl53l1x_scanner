#ifndef __SHARED_CONF__
#define __SHARED_CONF__

// Constants
#define ARDUINO_SERIAL_BAUD_RATE            (9600)
#define ARDUINO_INPUT_STRING_SIZE           (24)
#define LASER_DEVICE_ADDR                   (0x52)
#define LASER_WIRE_CLOCK                    (400000)
#define LASER_ROI_MINCENTERED_TOPLEFTX      (6)
#define LASER_ROI_MINCENTERED_TOPLEFTY      (9) // 15
#define LASER_ROI_MINCENTERED_BOTRIGHTX     (9)
#define LASER_ROI_MINCENTERED_BOTRIGHTY     (6) // 0
#define LASER_DISTANCE_MODE_SHORT           (1)
#define LASER_DISTANCE_MODE_MEDIUM          (2)
#define LASER_DISTANCE_MODE_LONG            (3)
#define LASER_VERTICAL_POS_BIAS             (-6)
#define SCAN_MODE_2D_LASERSCAN              (1)
#define SCAN_MODE_2D_POINTCLOUD             (2)
#define SCAN_MODE_3D_POINTCLOUD             (3)
#define SCAN_MODE_CAM_DEPTHIMAGE            (4)


// Data Type
#ifdef __THIS_IS_THE_DRIVER__
    #define SHARED_INT_TYPE     int
    #define SHARED_LONG_TYPE    int
    #define SHARED_FLOAT_TYPE   double
#endif
#ifdef __THIS_IS_THE_SCANNER__
    #define SHARED_INT_TYPE     int
    #define SHARED_LONG_TYPE    long
    #define SHARED_FLOAT_TYPE   float
#endif


//------------------- PARAMETERS (in order) -------------------------------------------------------
SHARED_INT_TYPE   p_stepper_pin_1                                               = 9;
SHARED_INT_TYPE   p_stepper_pin_2                                               = 10;
SHARED_INT_TYPE   p_stepper_pin_3                                               = 11;
SHARED_INT_TYPE   p_stepper_pin_4                                               = 12;
SHARED_INT_TYPE   p_stepper_delay                                               = 2250;
SHARED_FLOAT_TYPE p_stepper_horizontal_angle_per_step                           = 0.003067962;
SHARED_INT_TYPE   p_stepper_step_min                                            = -512;
SHARED_INT_TYPE   p_stepper_step_max                                            = +512;
SHARED_FLOAT_TYPE p_laser_range_min                                             = 0.04;
SHARED_FLOAT_TYPE p_laser_range_max                                             = 4.00;
SHARED_INT_TYPE   p_laser_roi_topleftx                                          = LASER_ROI_MINCENTERED_TOPLEFTX;
SHARED_INT_TYPE   p_laser_roi_toplefty                                          = LASER_ROI_MINCENTERED_TOPLEFTY;
SHARED_INT_TYPE   p_laser_roi_botrightx                                         = LASER_ROI_MINCENTERED_BOTRIGHTX;
SHARED_INT_TYPE   p_laser_roi_botrighty                                         = LASER_ROI_MINCENTERED_BOTRIGHTY;
SHARED_INT_TYPE   p_laser_distance_mode                                         = LASER_DISTANCE_MODE_LONG;
SHARED_FLOAT_TYPE p_laser_vertical_angle_per_roi_cell                           = 0.035;
SHARED_LONG_TYPE  p_laser_measurement_timing_budget_micro_seconds               = 50000;
SHARED_INT_TYPE   p_laser_inter_measurement_period_milli_seconds                = 50;
SHARED_INT_TYPE   p_scanner_mode                                                = SCAN_MODE_2D_POINTCLOUD;
SHARED_INT_TYPE   p_scanner_3d_vertical_steps_per_scan                          = 1;
SHARED_INT_TYPE   p_scanner_horizontal_steps_per_scan                           = 32;
SHARED_INT_TYPE   p_scanner_rewind                                              = 1;
SHARED_INT_TYPE   p_scanner_calibration_max_value                               = 500;
SHARED_INT_TYPE   p_adaptive_resolution_enable                                  = 1;
SHARED_FLOAT_TYPE p_adaptive_resolution_coef_0                                  = 0.05189296;
SHARED_FLOAT_TYPE p_adaptive_resolution_coef_1                                  = -0.00166131;

#ifdef __THIS_IS_THE_DRIVER__
    #define STR_PRINT(x) #x
    #define PROCESS_VARIABLE_INT(id, v) { priv_nh->param(#v, v, v); sprintf(write_buffer, "V %d %d\n", id, v); ser.write(write_buffer); ser.flush(); /*ROS_INFO(write_buffer);*/}
    #define PROCESS_VARIABLE_LONG(id, v) { priv_nh->param(#v, v, v); sprintf(write_buffer, "V %d %d\n", id, v); ser.write(write_buffer); ser.flush(); /*ROS_INFO(write_buffer);*/}
    #define PROCESS_VARIABLE_FLOAT(id, v) { priv_nh->param(#v, v, v); sprintf(write_buffer, "V %d %.8f\n", id, v); ser.write(write_buffer); ser.flush(); /*ROS_INFO(write_buffer);*/} // 
#endif
#ifdef __THIS_IS_THE_SCANNER__
    #define PROCESS_VARIABLE_INT(tmp, v) {token = strtok(NULL, " "); v = atoi(token); Serial.print(F("#   ")); Serial.print(F(#v)); Serial.print(F(": ")); Serial.println(v);}
    #define PROCESS_VARIABLE_LONG(tmp, v) {token = strtok(NULL, " "); v = atol(token); Serial.print(F("#   ")); Serial.print(F(#v)); Serial.print(F(": ")); Serial.println(v);}
    #define PROCESS_VARIABLE_FLOAT(tmp, v) {token = strtok(NULL, " "); v = atof(token); Serial.print(F("#   ")); Serial.print(F(#v)); Serial.print(F(": ")); Serial.println(v, 8);}
#endif

#define PROCESS_PARAMETER_SIZE (25+1)
#define PROCESS_PARAMETER(id) {                                                                          \
    switch(id)                                                                                           \
    {                                                                                                    \
      case 0:  PROCESS_VARIABLE_INT   (id, p_stepper_pin_1);                                 break;      \
      case 1:  PROCESS_VARIABLE_INT   (id, p_stepper_pin_2);                                 break;      \
      case 2:  PROCESS_VARIABLE_INT   (id, p_stepper_pin_3);                                 break;      \
      case 3:  PROCESS_VARIABLE_INT   (id, p_stepper_pin_4);                                 break;      \
      case 4:  PROCESS_VARIABLE_INT   (id, p_stepper_delay);                                 break;      \
      case 5:  PROCESS_VARIABLE_INT   (id, p_stepper_step_min);                              break;      \
      case 6:  PROCESS_VARIABLE_INT   (id, p_stepper_step_max);                              break;      \
      case 7:  PROCESS_VARIABLE_INT   (id, p_laser_roi_topleftx);                            break;      \
      case 8:  PROCESS_VARIABLE_INT   (id, p_laser_roi_toplefty);                            break;      \
      case 9:  PROCESS_VARIABLE_INT   (id, p_laser_roi_botrightx);                           break;      \
      case 10: PROCESS_VARIABLE_INT   (id, p_laser_roi_botrighty);                           break;      \
      case 11: PROCESS_VARIABLE_INT   (id, p_laser_distance_mode);                           break;      \
      case 12: PROCESS_VARIABLE_LONG  (id, p_laser_measurement_timing_budget_micro_seconds); break;      \
      case 13: PROCESS_VARIABLE_INT   (id, p_laser_inter_measurement_period_milli_seconds);  break;      \
      case 14: PROCESS_VARIABLE_INT   (id, p_scanner_mode);                                  break;      \
      case 15: PROCESS_VARIABLE_INT   (id, p_scanner_3d_vertical_steps_per_scan);            break;      \
      case 16: PROCESS_VARIABLE_INT   (id, p_scanner_horizontal_steps_per_scan);             break;      \
      case 17: PROCESS_VARIABLE_INT   (id, p_scanner_rewind);                                break;      \
      case 18: PROCESS_VARIABLE_INT   (id, p_scanner_calibration_max_value);                 break;      \
      case 19: PROCESS_VARIABLE_FLOAT (id, p_stepper_horizontal_angle_per_step);             break;      \
      case 20: PROCESS_VARIABLE_FLOAT (id, p_laser_range_min);                               break;      \
      case 21: PROCESS_VARIABLE_FLOAT (id, p_laser_range_max);                               break;      \
      case 22: PROCESS_VARIABLE_FLOAT (id, p_laser_vertical_angle_per_roi_cell);             break;      \
      case 23: PROCESS_VARIABLE_INT   (id, p_adaptive_resolution_enable);                    break;      \
      case 24: PROCESS_VARIABLE_FLOAT (id, p_adaptive_resolution_coef_0);                    break;      \
      case 25: PROCESS_VARIABLE_FLOAT (id, p_adaptive_resolution_coef_1);                    break;      \
    }                                                                                                    \
} // DO NOT FORGET TO UPDATE (PROCESS_PARAMETER_SIZE) !!!!

#endif /*__SHARED_CONF__*/
