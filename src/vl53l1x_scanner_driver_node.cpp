#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cstring>
#include <vector>
#include <algorithm>

#define __THIS_IS_THE_DRIVER__
#include "vl53l1x_scanner_arduino/shared_conf.h"

#define CALCULATE_INTENSITY(SIGNAL_RATE, AMBIENT_RATE) ( (SIGNAL_RATE) )

union PC_DATA_t
{
    struct
    {
        float x, y, z, range_status, intensity, signal_rate, ambient_rate;
    }structured;
    uint8_t raw[28];
};

union IMAGE_DATA_t
{
    struct
    {
        float r;
    }structured;
    uint8_t raw[4];
};

// Global Variables
serial::Serial ser;
char read_buffer[512];
char write_buffer[512];
ros::Publisher laser_pub;
bool is_laser_pub_advertised = false;
ros::Publisher pc_pub;
bool is_pc_pub_advertised = false;
ros::Publisher image_pub;
bool is_image_pub_advertised = false;
sensor_msgs::LaserScan laser_scan;
sensor_msgs::PointCloud2 pc_scan;
sensor_msgs::Image image_scan;
bool laser_scan_direction;
ros::NodeHandle *nh, *priv_nh;
ros::Time t_start, t_stop;

// All parameters with prefix "p_" are located in shared_conf.h
// Other parameters:
std::string     scanner_serial_port = "/dev/ttyACM0";
std::string     laserscan_frame = "laser";

void initialize();
void sendParameters();
void startScanning();
void shutdownDevice();
char* readLineFromDevice();
void process2D(char *line);
void process3D(char *line);
void processDepth(char *line);
void processInput();


int main (int argc, char** argv)
{
    ros::init(argc, argv, "vl53l1x_scanner_driver_node");
    nh = new ros::NodeHandle();
    priv_nh = new ros::NodeHandle("~"); // will be used in shared_conf.h

    priv_nh->param("scanner_serial_port", scanner_serial_port, scanner_serial_port);
    priv_nh->param("laserscan_frame", laserscan_frame, laserscan_frame);

    // Open serial port
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
    while(ser.available()) ser.read(1)[0]; 

    // Send Parameters -> Initialize the Device -> Start Scanning
    ros::Duration(3.0).sleep(); // Wait for arduino
    sendParameters();
    ros::Duration(1.0).sleep(); // Wait for parameter responses
    initialize();
    ros::Duration(1.0).sleep(); // Wait for initialization
    startScanning();

    while (ros::ok())
    {
        processInput();
    }

    shutdownDevice();

}

void processInput()
{
    char *line = readLineFromDevice();
    if (line == NULL) return;
    if (line[0] == '#')
    {
        ROS_WARN("%s", line);
    }
    else
    {
        ROS_INFO(line);
        switch (p_scanner_mode)
        {
            case SCAN_MODE_2D_LASERSCAN:    process2D(line);    break;
            case SCAN_MODE_2D_POINTCLOUD:   process3D(line);    break;
            case SCAN_MODE_3D_POINTCLOUD:   process3D(line);    break;
            case SCAN_MODE_CAM_DEPTHIMAGE:  processDepth(line); break;
        }
    }
}

void sendParameters()
{
    for (int i=0; i<PROCESS_PARAMETER_SIZE; i++)
    {
        PROCESS_PARAMETER(i);
        ros::Duration(0.1).sleep(); // need to wait because it causes overflow in arduino
        processInput();
        ros::Duration(0.1).sleep(); // need to wait because it causes overflow in arduino
    }
    processInput();
}

void initialize()
{
    sprintf(write_buffer, "I \n");
    ser.write(write_buffer); // Initialize
    ser.flush();
}

void startScanning()
{
    ser.write("S\n"); // Start Scanning
    ser.flush();
}

void shutdownDevice()
{
    ser.write("P \n"); // Shutdown
    ser.flush();
}

char* readLineFromDevice()
{
    if (ser.available())
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
    return NULL;
}

void process2D(char *line)
{
    static int laser_seq = 0;
    if (line[0] == '$') // Publish old and construct new laser data
    {
        if (laser_scan.ranges.size() > 0 && ros::ok())
        {
            t_stop = ros::Time::now();
            ROS_WARN_STREAM("Full-Scan period: " << t_stop-t_start);

            // Fix data
            if (!laser_scan_direction)
            {
                std::reverse(laser_scan.ranges.begin(), laser_scan.ranges.end());
                std::reverse(laser_scan.intensities.begin(), laser_scan.intensities.end());
            }
            //laser_scan.scan_time = ros::Duration(t_stop-t_start);

            // Publish data
            if (!is_laser_pub_advertised)
            {
                laser_pub = nh->advertise<sensor_msgs::LaserScan>("scan", 2, true);
                is_laser_pub_advertised = true;
            }
            laser_pub.publish(laser_scan);
        }

        laser_scan_direction = (line[1] == '+'); // scan direction for new laser data

        // Create ROS message
        laser_scan.header.seq = laser_seq++;
        laser_scan.header.stamp = ros::Time::now();
        laser_scan.header.frame_id = laserscan_frame;
        laser_scan.angle_min = p_stepper_horizontal_angle_per_step * p_stepper_step_min;
        laser_scan.angle_max = p_stepper_horizontal_angle_per_step * p_stepper_step_max;
        laser_scan.angle_increment = p_stepper_horizontal_angle_per_step * p_scanner_horizontal_steps;
        laser_scan.time_increment = 0; // not usable since scan time varies among measurements
        laser_scan.scan_time = 0; // will be updated
        laser_scan.range_min = p_laser_range_min;
        laser_scan.range_max = p_laser_range_max;
        laser_scan.ranges.clear();
        laser_scan.intensities.clear();

        t_start = ros::Time::now();
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
            t_stop = ros::Time::now();
            ROS_WARN_STREAM("Full-Scan period: " << t_stop-t_start);

            // Publish data
            if (!is_pc_pub_advertised)
            {
                pc_pub = nh->advertise<sensor_msgs::PointCloud2>("vl53l1x_points", 2, true);
                is_pc_pub_advertised = true;
            }
            pc_pub.publish(pc_scan);
        }

        // Create ROS message
        pc_scan.header.seq = pc_seq++;
        pc_scan.header.stamp = ros::Time::now();
        pc_scan.header.frame_id = laserscan_frame;
        pc_scan.height = 1; // 2D structure of the point cloud. If the cloud is unordered, height is
        pc_scan.width = 0;  // 1 and width is the length of the point cloud.
        pc_scan.fields.clear(); // Describes the channels and their layout in the binary data blob.

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

        t_start = ros::Time::now();
    }
    else
    {
        int horizontal_pos, vertical_pos, range, status;
        float sig1, sig2;
        sscanf(line, "%d %d %d %d %f %f", &horizontal_pos, &vertical_pos, &range, &status, &sig1, &sig2);

        float r = ((float)range)/1000.0;
        float theta = p_stepper_horizontal_angle_per_step * ((float)horizontal_pos);
        float beta = p_laser_vertical_angle_per_roi_cell * ((float)vertical_pos);

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

void processDepth(char *line)
{
    static int image_seq = 0;
    if (line[0] == '$') // Publish old and construct new laser data
    {
        if (image_scan.data.size() > 0 && ros::ok())
        {
            t_stop = ros::Time::now();
            ROS_WARN_STREAM("Full-Scan period: " << t_stop-t_start);

            // Publish data
            if (!is_image_pub_advertised)
            {
                image_pub = nh->advertise<sensor_msgs::Image>("/vl53l1x/image/depth", 2, true);
                is_image_pub_advertised = true;
            }
            image_pub.publish(image_scan);
        }

        // Create ROS message
        image_scan.header.seq = image_seq++;
        image_scan.header.stamp = ros::Time::now();
        image_scan.header.frame_id = laserscan_frame;

        image_scan.height = 12/p_scanner_vertical_steps+1 ;
        image_scan.width = 12/p_scanner_horizontal_steps+1;

        image_scan.encoding = "32FC1";
        image_scan.is_bigendian = false;
        image_scan.step = image_scan.width * sizeof(float);
        image_scan.data.clear();

        t_start = ros::Time::now();
    }
    else
    {
        int horizontal_pos, vertical_pos, range, status;
        float sig1, sig2;
        sscanf(line, "%d %d %d %d %f %f", &horizontal_pos, &vertical_pos, &range, &status, &sig1, &sig2);

        float r = ((float)range)/1000.0;

        IMAGE_DATA_t image_data;
        image_data.structured.r = r;

        for (int i=0; i<sizeof(image_data.raw); i++)
        {
            image_scan.data.push_back(image_data.raw[i]);
        }
    }
}