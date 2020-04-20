#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <cstring>
#include <vector>
#include <algorithm>

#define CALCULATE_INTENSITY(SIGNAL_RATE, AMBIENT_RATE) ( (SIGNAL_RATE) )

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

union PC_DATA_t
{
    struct
    {
        float x, y, z, range_status, intensity, signal_rate, ambient_rate;
    }structured;
    uint8_t raw[28];
};

// Global Variables
serial::Serial ser;
char read_buffer[512];
char write_buffer[512];
ros::Publisher laser_pub;
ros::Publisher pc_pub;
sensor_msgs::LaserScan laser_scan;
sensor_msgs::PointCloud2 pc_scan; 
bool laser_scan_direction;


// Hardware specific
std::string scanner_serial_port = "/dev/ttyACM0";
double stepper_horizontal_angle_per_step = 0.003067962; // pi/1024 TODO
double laser_vertical_angle_per_roi_cell = 0.035; // TODO
double laser_range_min = 0.0;
double laser_range_max = 4.0; // TODO


// ROS specific
std::string laserscan_frame = "laser";


// Scanner specific
int stepper_pin_1 = 9;
int stepper_pin_2 = 10;
int stepper_pin_3 = 11;
int stepper_pin_4 = 12;
int stepper_delay = 2250; // my stepper motor misses step with 2000. So I added extra 250
int stepper_step_min = -300;
int stepper_step_max = +300;
int laser_roi_topleftx = 6;
int laser_roi_toplefty = 9;
int laser_roi_botrightx = 9;
int laser_roi_botrighty = 6;
int laser_distance_mode = 3; // ADAPTIVE(0), short(1), medium(2), long(3)
int laser_measurement_timing_budget_micro_seconds = 500000;
int laser_inter_measurement_period_milli_seconds = 50;
int scanner_mode = 1;
int scanner_3d_vertical_steps_per_scan = 1;
int scanner_horizontal_steps_per_scan = 20;
int scanner_rewind = 1;
int scanner_calibration_max_value = 500;


void sendParameters();
void startScanning();
void stopScanning();
char* readLineFromDevice();
void process2D(char *line);
void process3D(char *line);


int main (int argc, char** argv)
{
    ros::init(argc, argv, "vl53l1x_scanner_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    // Hardware specific
    priv_nh.param("scanner_serial_port", scanner_serial_port, scanner_serial_port);
    priv_nh.param("stepper_horizontal_angle_per_step", stepper_horizontal_angle_per_step, stepper_horizontal_angle_per_step);
    priv_nh.param("laser_vertical_angle_per_roi_cell", laser_vertical_angle_per_roi_cell, laser_vertical_angle_per_roi_cell);
    priv_nh.param("laser_range_min", laser_range_min, laser_range_min);
    priv_nh.param("laser_range_max", laser_range_max, laser_range_max);

    // ROS specific
    priv_nh.param("laserscan_frame", laserscan_frame, laserscan_frame);

    // Scanner specific (will be sent to the scanner)
    priv_nh.param("stepper_pin_1", stepper_pin_1, stepper_pin_1);
    priv_nh.param("stepper_pin_2", stepper_pin_2, stepper_pin_2);
    priv_nh.param("stepper_pin_3", stepper_pin_3, stepper_pin_3);
    priv_nh.param("stepper_pin_3", stepper_pin_3, stepper_pin_3);
    priv_nh.param("stepper_delay", stepper_delay, stepper_delay);
    priv_nh.param("stepper_step_min", stepper_step_min, stepper_step_min);
    priv_nh.param("stepper_step_max", stepper_step_max, stepper_step_max);
    priv_nh.param("laser_roi_topleftx", laser_roi_topleftx, laser_roi_topleftx);
    priv_nh.param("laser_roi_toplefty", laser_roi_toplefty, laser_roi_toplefty);
    priv_nh.param("laser_roi_botrightx", laser_roi_botrightx, laser_roi_botrightx);
    priv_nh.param("laser_roi_botrighty", laser_roi_botrighty, laser_roi_botrighty);
    priv_nh.param("laser_distance_mode", laser_distance_mode, laser_distance_mode);
    priv_nh.param("laser_measurement_timing_budget_micro_seconds", laser_measurement_timing_budget_micro_seconds, laser_measurement_timing_budget_micro_seconds);
    priv_nh.param("laser_inter_measurement_period_milli_seconds", laser_inter_measurement_period_milli_seconds, laser_inter_measurement_period_milli_seconds);
    priv_nh.param("scanner_mode", scanner_mode, scanner_mode);
    priv_nh.param("scanner_3d_vertical_steps_per_scan", scanner_3d_vertical_steps_per_scan, scanner_3d_vertical_steps_per_scan);
    priv_nh.param("scanner_horizontal_steps_per_scan", scanner_horizontal_steps_per_scan, scanner_horizontal_steps_per_scan);
    priv_nh.param("scanner_rewind", scanner_rewind, scanner_rewind);
    priv_nh.param("scanner_calibration_max_value", scanner_calibration_max_value, scanner_calibration_max_value);

    switch (scanner_mode)
    {

    }
    if (scanner_mode)
    {
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("vl53l1x_points", 2);
    }
    else
    {
        laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 2);
    }

    try
    {
        ser.setPort(scanner_serial_port);
        ser.setBaudrate(ARDUINO_SERIAL_BAUD_RATE);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    // Flush old buffer
    while(ser.available())
    {
        ser.read(1)[0];
    }

    ros::Duration(3.0).sleep(); // Wait for arduino

    sendParameters();

    ros::Duration(1.0).sleep(); // Wait for parameter responses

    startScanning();

    while (ros::ok())
    {
        char *line = readLineFromDevice();
        if (line == NULL) continue;

        if (line[0] == '#')
        {
            ROS_WARN("%s", line);
        }
        else if (scanner_mode)
        {
            ROS_ERROR("%s", line);
            process3D(line);
        }
        else
        {
            ROS_ERROR("%s", line);
            process2D(line);
        }
    }

    stopScanning();

}

void sendParameters()
{
    /*
    sprintf(write_buffer, "I %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",   stepper_pin_1, \
                                                                                            stepper_pin_2, \
                                                                                            stepper_pin_3, \
                                                                                            stepper_pin_4, \
                                                                                            stepper_delay, \
                                                                                            stepper_step_min, \
                                                                                            stepper_step_max, \
                                                                                            laser_roi_topleftx, \
                                                                                            laser_roi_toplefty, \
                                                                                            laser_roi_botrightx, \
                                                                                            laser_roi_botrighty, \
                                                                                            laser_distance_mode, \
                                                                                            laser_measurement_timing_budget_micro_seconds, \
                                                                                            laser_inter_measurement_period_milli_seconds, \
                                                                                            scanner_mode, \
                                                                                            scanner_3d_vertical_steps_per_scan, \
                                                                                            scanner_horizontal_steps_per_scan, \
                                                                                            scanner_rewind, \
                                                                                            scanner_calibration_max_value);
    */
    sprintf(write_buffer, "I\n");
    ROS_INFO("WRITE CHARS: %d", strlen(write_buffer));
    if (strlen(write_buffer)+1 > ARDUINO_INPUT_STRING_SIZE)
    {
        ROS_FATAL("OVERFLOW ARDUINO_INPUT_STRING_SIZE!");
    }
    ser.write(write_buffer); // Initialize
    ser.flush();
}

void startScanning()
{
    ser.write("S\n"); // Start Scanning
    ser.flush();
}

void stopScanning()
{
    ser.write("P\n"); // Shutdown
    ser.flush();
}

char* readLineFromDevice()
{
    bool is_line_complete = false;
    int i=0;
    ros::Rate rate(100);
    while(ros::ok() && !is_line_complete)
    {
        if(ser.available())
        {
            read_buffer[i] = ser.read(1)[0];
            if (read_buffer[i] == '\n')
            {
                read_buffer[i] = '\0';
                is_line_complete = true;
            }
            i++;
        }
        else
        {
            rate.sleep();
        }
    }
    if (is_line_complete)
    {
        return read_buffer; 
    }
    else
    {
        ROS_WARN("Exit interrupt received while reading from device");
        return NULL;
    }
}

void process2D(char *line)
{
    static int laser_seq = 0;
    if (line[0] == '$') // Publish old and construct new laser data
    {
        if (laser_scan.ranges.size() > 0 && ros::ok())
        {
            if (!laser_scan_direction)
            {
                std::reverse(laser_scan.ranges.begin(), laser_scan.ranges.end());
                std::reverse(laser_scan.intensities.begin(), laser_scan.intensities.end());
            }
            laser_pub.publish(laser_scan);
        }

        laser_scan_direction = (line[1] == '+'); // scan direction for new laser data

        laser_scan.header.seq = laser_seq++;
        laser_scan.header.stamp = ros::Time::now();
        laser_scan.header.frame_id = laserscan_frame;
        laser_scan.angle_min = stepper_horizontal_angle_per_step * stepper_step_min;
        laser_scan.angle_max = stepper_horizontal_angle_per_step * stepper_step_max;
        laser_scan.angle_increment = stepper_horizontal_angle_per_step * scanner_horizontal_steps_per_scan;
        laser_scan.time_increment = 0;                                                              // TODO
        laser_scan.scan_time = 0;                                                                   // TODO
        laser_scan.range_min = laser_range_min;
        laser_scan.range_max = laser_range_max;
        laser_scan.ranges.clear();
        laser_scan.intensities.clear();
    }
    else
    {
        int horizontal_pos, vertical_pos, range, status;
        float sig1, sig2;
        sscanf(line, "%d %d %d %d %f %f", &horizontal_pos, &vertical_pos, &range, &status, &sig1, &sig2);

        laser_scan.ranges.push_back( ((float)range)/1000.0 );
        laser_scan.intensities.push_back( CALCULATE_INTENSITY(sig1, sig2) );
    }

}

void process3D(char *line)
{
    static int pc_seq = 0;
    if (line[0] == '$') // Publish old and construct new laser data
    {
        if (pc_scan.data.size() > 0 && ros::ok())
        {
            pc_pub.publish(pc_scan);
        }

        pc_scan.header.seq = pc_seq++;
        pc_scan.header.stamp = ros::Time::now();
        pc_scan.header.frame_id = laserscan_frame;

        // 2D structure of the point cloud. If the cloud is unordered, height is
        // 1 and width is the length of the point cloud.
        pc_scan.height = 1;
        pc_scan.width = 0;

        // Describes the channels and their layout in the binary data blob.
        pc_scan.fields.clear();

        std::string field_names[7] = {"x", "y", "z", "range_status", "intensity", "signal_rate", "ambient_rate"};
        for (int i=0; i<7; i++)
        {
            sensor_msgs::PointField point_field;
            point_field.name = field_names[i];
            point_field.offset = i*sizeof(float);
            point_field.datatype = sensor_msgs::PointField::FLOAT32;
            point_field.count = 1;
            pc_scan.fields.push_back(point_field);
        }

        pc_scan.is_bigendian = false;
        pc_scan.point_step = pc_scan.fields.size() * sizeof(float);
        pc_scan.row_step = pc_scan.fields.size() * sizeof(float);    // Length of a row in bytes
        pc_scan.data.clear();
        pc_scan.is_dense = false; // TODO: True if there are no invalid points
    }
    else
    {
        int horizontal_pos, vertical_pos, range, status;
        float sig1, sig2;
        sscanf(line, "%d %d %d %d %f %f", &horizontal_pos, &vertical_pos, &range, &status, &sig1, &sig2);

        float r = ((float)range)/1000.0;
        float theta = stepper_horizontal_angle_per_step * ((float)horizontal_pos);
        float beta = laser_vertical_angle_per_roi_cell * ((float)vertical_pos);

        PC_DATA_t pc_data;
        pc_data.structured.x = r * cos(beta) * cos(theta);
        pc_data.structured.y = r * cos(beta) * sin(theta);
        pc_data.structured.z = r * sin(beta);
        pc_data.structured.range_status = status;
        pc_data.structured.intensity = CALCULATE_INTENSITY(sig1, sig2);
        pc_data.structured.signal_rate = sig1;
        pc_data.structured.ambient_rate = sig2;

        //ROS_INFO("%f %f %f", pc_data.structured.x,pc_data.structured.y,pc_data.structured.z);

        for (int i=0; i<sizeof(pc_data.raw); i++)
        {
            pc_scan.data.push_back(pc_data.raw[i]);
        }

         pc_scan.width++;
    }
}