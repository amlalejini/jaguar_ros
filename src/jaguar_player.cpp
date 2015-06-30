#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <jaguar_ros/MotorInfo.h>
#include <jaguar_ros/JaguarMotorSensorData.h>
#include <jaguar_ros/JaguarMotionBoardInfo.h>
#include <jaguar_ros/JaguarMotorTemps.h>
#include <jaguar_ros/JaguarBatteryInfo.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <map>

#include "DrRobotMotionSensorDriver.hpp"

using std::string;
using std::map;

using namespace DrRobot_MotionSensorDriver;

/*
    MODULE TODO:
        - Potentially take out serial stuff? Jaguar only ever uses network connection in their code
        - openSerial is never used...
*/

////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////
// Networking
string DEFAULT_JAGUAR_NETWORK_IP = "192.168.0.70";
int DEFAULT_JAGUAR_NETWORK_PORT = 10001;
// Topics
string DEFAULT_DRIVE_VEL_TOPIC = "cmd_vel";
string DEFAULT_FRONT_FLIPPER_CONTROL_TOPIC = "front_flipper_cmds";
string DEFAULT_REAR_FLIPPER_CONTROL_TOPIC = "rear_flipper_cmds";
string DEFAULT_HEADLIGHT_CONTROL_TOPIC = "headlight_cmds";
string DEFAULT_MOTOR_SENSORS_TOPIC = "jaguar_motor_sensors";
string DEFAULT_MOTION_BOARD_INFO_TOPIC = "jaguar_motion_board_info";
string DEFAULT_BATTERY_INFO_TOPIC = "jaguar_battery_info";
string DEFAULT_MOTOR_TEMPERATURE_SENSORS_TOPIC = "jaguar_motor_temperatures";
// Motor Constants
//   - Drive motor constants
int DEFAULT_DRIVE_MOTOR_DIRECTION = 1;
double DEFAULT_DRIVE_MOTOR_MAX_SPEED = 1.0;
double DEFAULT_DRIVE_MOTOR_MIN_SPEED = 0.1;
//  - Flipper motor constants
int DEFAULT_FLIPPER_MOTOR_DIRECTION = 1;
double DEFAULT_FLIPPER_MOTOR_MAX_SPEED = 1.0;
double DEFAULT_FLIPPER_MOTOR_MIN_SPEED = 0.1;
//  - IDs
int DEFAULT_FRONT_FLIPPER_ID = 0;
int DEFAULT_REAR_FLIPPER_ID = 1;
int DEFAULT_PORT_DRIVE_ID = 3;
int DEFAULT_STARTBOARD_DRIVE_ID = 4;
// Sensor Constants
//   - Motion board sensor
int DEFAULT_MOTION_BOARD_HEAT_SENSOR_CNT = 0;
////////////////////////////////////////////////////////

class JaguarPlayer {
    /*
        This class defines the ROS node that interfaces with the DrRobot Jaguar's
        motorola evaluation board.  This board controls robot PWM control, motor encoder data reportings,
        the front headlights, and <OTHER SENSORS TO BE DETERMINED>
    */
public:
    JaguarPlayer(void);         // Constuctor
    ~JaguarPlayer(void);        // Destructor
    
    void run(void);         // Main run function for jaguar player node

private:
    
    // Jaguar Driver
    DrRobotMotionSensorDriver* jaguar_driver;

    ros::NodeHandle node_handle;
    // ROS subscribers
    ros::Subscriber drive_vel_sub;
    ros::Subscriber front_flipper_cmds_sub;
    ros::Subscriber rear_flipper_cmds_sub;
    ros::Subscriber headlight_cmds_sub;
    // ROS Publishers
    ros::Publisher motor_info_pub;
    ros::Publisher motion_board_info_pub;
    ros::Publisher motor_temps_pub;
    ros::Publisher battery_info_pub;

    // Networking variables
    string jaguar_network_ip;
    int jaguar_network_port;
    string jaguar_serial_port;
    struct DrRobotMotionConfig jaguar_driver_config;
    // Topic names
    string drive_vel_topic;
    string front_flipper_ctrl_topic;
    string rear_flipper_ctrl_topic;
    string headlight_ctrl_topic;
    string motor_sensors_topic;
    string motion_board_info_topic;
    string motor_temperature_sensors_topic;
    string battery_info_topic;
    // Motor parameters (can't use floats instead of doubles because of ros param server compatibility issue)
    double drive_max_speed;
    double drive_min_speed;
    int drive_motor_direction;
    double flipper_max_speed;
    double flipper_min_speed;
    int flipper_motor_direction;
    map<string, int> motor_ids;
    // Sensor data variables
    int motion_board_heat_sensor_cnt;
    struct MotorSensorData motor_sensor_data; // documented in DrRobotMotionSensorDriver.hpp
    struct StandardSensorData standard_sensor_data;
    struct CustomSensorData custom_sensor_data;

    void connect(void);
    void update(void);
    double drrobotRawMotorTemperatureToCelsius(double adValue);

    void driveVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void frontFlipperCallback(const std_msgs::Float32::ConstPtr& msg);
    void rearFlipperCallback(const std_msgs::Float32::ConstPtr& msg);
    void headlightCallback(const std_msgs::Bool::ConstPtr& msg);
};

JaguarPlayer::JaguarPlayer() {
    /*
     *   Jaguar Player default contructor.
     */

    ///////////////////////////////////////////////////////
    // Load parameters from parameter server
    ///////////////////////////////////////////////////////
    // - Load Networking information
    node_handle.param<string>("player/network_ip", jaguar_network_ip, DEFAULT_JAGUAR_NETWORK_IP);
    node_handle.param<int>("player/network_port", jaguar_network_port, DEFAULT_JAGUAR_NETWORK_PORT);
    // - Load topic names
    node_handle.param<string>("robot_control_topics/drive_control", drive_vel_topic, DEFAULT_DRIVE_VEL_TOPIC);
    node_handle.param<string>("robot_control_topics/front_flipper_control", front_flipper_ctrl_topic, DEFAULT_FRONT_FLIPPER_CONTROL_TOPIC);
    node_handle.param<string>("robot_control_topics/rear_flipper_control", rear_flipper_ctrl_topic, DEFAULT_REAR_FLIPPER_CONTROL_TOPIC);
    node_handle.param<string>("robot_control_topics/headlight_control", headlight_ctrl_topic, DEFAULT_HEADLIGHT_CONTROL_TOPIC);
    node_handle.param<string>("sensors/motor_sensors/topic", motor_sensors_topic, DEFAULT_MOTOR_SENSORS_TOPIC);
    node_handle.param<string>("sensors/motion_board/topic", motion_board_info_topic, DEFAULT_MOTION_BOARD_INFO_TOPIC);
    node_handle.param<string>("sensors/motor_temperature/topic", motor_temperature_sensors_topic, DEFAULT_MOTOR_TEMPERATURE_SENSORS_TOPIC);
    node_handle.param<string>("sensors/battery", battery_info_topic, DEFAULT_BATTERY_INFO_TOPIC);
    // - Load Motor parameters
    //   - Drive
    node_handle.param<double>("motors/drive/max_speed", drive_max_speed, DEFAULT_DRIVE_MOTOR_MAX_SPEED);
    node_handle.param<double>("motors/drive/min_speed", drive_min_speed, DEFAULT_DRIVE_MOTOR_MIN_SPEED);
    node_handle.param<int>("motors/drive/motor_direction", drive_motor_direction, DEFAULT_DRIVE_MOTOR_DIRECTION);
    //   - Flippers
    node_handle.param<double>("motors/flippers/max_speed", flipper_max_speed, DEFAULT_FLIPPER_MOTOR_MAX_SPEED);
    node_handle.param<double>("motors/flippers/min_speed", flipper_min_speed, DEFAULT_FLIPPER_MOTOR_MIN_SPEED);
    node_handle.param<int>("motors/flippers/motor_direction", flipper_motor_direction, DEFAULT_FLIPPER_MOTOR_DIRECTION);
    //   - IDs
    node_handle.param<int>("motors/ids/front_flipper", motor_ids["FRONT_FLIPPER"], DEFAULT_FRONT_FLIPPER_ID);
    node_handle.param<int>("motors/ids/rear_flipper", motor_ids["REAR_FLIPPER"], DEFAULT_REAR_FLIPPER_ID);
    node_handle.param<int>("motors/ids/port_drive", motor_ids["PORT_DRIVE"], DEFAULT_PORT_DRIVE_ID);
    node_handle.param<int>("motors/ids/starboard_drive", motor_ids["STARBOARD_DRIVE"], DEFAULT_STARTBOARD_DRIVE_ID);
    // - Load sensor constants
    node_handle.param<int>("sensors/motion_board/heat_sensor_cnt", motion_board_heat_sensor_cnt, DEFAULT_MOTION_BOARD_HEAT_SENSOR_CNT);
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // Setup Jaguar driver configuration
    ///////////////////////////////////////////////////////
    //  - Set communication type (network or serial)
    // TODO: get rid of commMethod, board type, and serialPortName when Lucas' code is merged with mine.
    jaguar_driver_config.commMethod = Network;
    // - Set robot type (Jaguar)
    jaguar_driver_config.boardType = Jaguar;
    // - Set jaguar player network port number
    jaguar_driver_config.portNum = jaguar_network_port;
    // - Set jaguar player IP (IP of motorola eval board)
    strcpy(jaguar_driver_config.robotIP, jaguar_network_ip.c_str());
    // - Set jaguar player serial port 
    strcpy(jaguar_driver_config.serialPortName, "garbage");
    ///////////////////////////////////////////////////////
    
    ///////////////////////////////////////////////////////
    // Create Jaguar Driver
    ///////////////////////////////////////////////////////
    jaguar_driver = new DrRobotMotionSensorDriver();
    jaguar_driver->setDrRobotMotionDriverConfig(&jaguar_driver_config);
    
    ///////////////////////////////////////////////////////
    // Robot startup
    ///////////////////////////////////////////////////////    
    // - Connect to robot
    connect();
    
    ///////////////////////////////////////////////////////
    // Setup ROS Publishers
    ///////////////////////////////////////////////////////
    motor_info_pub = node_handle.advertise<jaguar_ros::JaguarMotorSensorData>(motor_sensors_topic, 1);
    motion_board_info_pub = node_handle.advertise<jaguar_ros::JaguarMotionBoardInfo>(motion_board_info_topic, 1);
    motor_temps_pub = node_handle.advertise<jaguar_ros::JaguarMotorTemps>(motor_temperature_sensors_topic, 1);
    battery_info_pub = node_handle.advertise<jaguar_ros::JaguarBatteryInfo>(battery_info_topic, 1);
    ///////////////////////////////////////////////////////
    // Setup ROS Subscribers
    ///////////////////////////////////////////////////////
    drive_vel_sub = node_handle.subscribe<geometry_msgs::Twist>(drive_vel_topic, 1000, boost::bind(&JaguarPlayer::driveVelCallback, this, _1));
    front_flipper_cmds_sub = node_handle.subscribe<std_msgs::Float32>(front_flipper_ctrl_topic, 1000, boost::bind(&JaguarPlayer::frontFlipperCallback, this, _1));
    rear_flipper_cmds_sub = node_handle.subscribe<std_msgs::Float32>(rear_flipper_ctrl_topic, 1000, boost::bind(&JaguarPlayer::rearFlipperCallback, this, _1));
    headlight_cmds_sub = node_handle.subscribe<std_msgs::Bool>(headlight_ctrl_topic, 1000, boost::bind(&JaguarPlayer::headlightCallback, this, _1));
}

JaguarPlayer::~JaguarPlayer() {
    /*
        JaguarPlayer destructor.
    */
    // clean up comms
    jaguar_driver->close();
    // clean up jaguar driver
    delete jaguar_driver;
}

void JaguarPlayer::connect() {
    /*
       This function attempts to connect to Jaguar robot.
       If it fails, it retries.
    */
    while (ros::ok()) {
        // attempt to connect to jaguar 
        int response = jaguar_driver->openNetwork(jaguar_driver_config.robotIP, jaguar_driver_config.portNum);
        if (response == 0) {
            // if successful, report success and break loop
            ROS_INFO("Connected to Jaguar at %s on port %d", jaguar_driver_config.robotIP, jaguar_driver_config.portNum);
            break;
        } else {
            // if failure, report failure then sleep and try again
            ROS_INFO("Failed to open network to Jaguar at %s on port %d.  Will continue trying.", jaguar_driver_config.robotIP, jaguar_driver_config.portNum);
            ros::Duration(3).sleep();
        }
    }
}

double JaguarPlayer::drrobotRawMotorTemperatureToCelsius(double adValue) {
    /*
        Given a raw value read in from drrobot Jaguar's motor temperature sensor, 
        return the temperature in celsius.
        Note: This function's contents are, for the most part, lifted directly 
        from the Windows C# Trans2Temperature(double adValue) function.
    */
    // I wish I had some documentation on the magic number palooza that is the below variable.
    // However, I unfortunately do not.
    double resTable[] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,
                        7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
    // Again, I wish I had some documentation.
    double tempTable[] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
    double FULLAD = 4095;
    double tempM;
    double k = (adValue / FULLAD);
    double resValue = 0;
    // I feel the need to go ahead and apologize again for what is about to happen.
    if (k != 1)
        resValue = 10000 * k / (1 - k);
    else
        resValue = resTable[0];

    int index = -1;
    if (resValue >= resTable[0])
        tempM = -20;
    else if (resValue <= resTable[24])
        tempM = 100;
    else {
        for (int i = 0; i < 24; i++) {
            if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1])) {
                index = i;
                break;
            }
        }
        if (index >= 0)
            tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
        else
            tempM = 0;
    }
    // I'm so sorry you had to read through that.
    return tempM;
}   

void JaguarPlayer::driveVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    /* 
      Gets called when message is received over the drive control topic (typically cmd_vel).
    */
     double lin_vel = msg->linear.x;
     double rot_vel = msg->angular.z;
     // PWM control (copied from original drrobot_player code)
     // TODO: Get rid of magic numbers (they are documented in motor sensor driver header)
     int linPWM = -drive_motor_direction * lin_vel * 16384 + 16384;
     int rotPWM = -drive_motor_direction * rot_vel * 16384 + 16384;
     if (linPWM > 32767) linPWM = 32767;
     if (linPWM < 0) linPWM = 0;
     if (rotPWM > 32767) rotPWM = 32767;
     if (rotPWM < 0) rotPWM = 0;
     // Send PWM commands to drivetrain motors
     jaguar_driver->sendMotorCtrlAllCmd(PWM, NOCONTROL, NOCONTROL, NOCONTROL, linPWM, rotPWM, NOCONTROL);
    
     // Velocity control (DOES NOT WORK...)
     //jaguar_driver->sendMotorCtrlAllCmd(Velocity, NOCONTROL, NOCONTROL, NOCONTROL, lin_vel, rot_vel, NOCONTROL);
}

void JaguarPlayer::frontFlipperCallback(const std_msgs::Float32::ConstPtr& msg) {
    /*
        Front flipper commands callback funtion.  This function is called every time
        commands are received over the front flipper control topic.
    */
    double flipper_vel = msg->data;
    // PWM Control (copied from origin drrobot_player node code)
    int flipperPWM = -flipper_motor_direction * flipper_vel * 16384 + 16384;
    if (flipperPWM > 32767) flipperPWM = 32767;
    if (flipperPWM < 0) flipperPWM = 0;
    // send PWM commands to flipper motors
    // TODO: switch this from all command to just single command
    jaguar_driver->sendMotorCtrlAllCmd(PWM, flipperPWM, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL);
    
}

void JaguarPlayer::rearFlipperCallback(const std_msgs::Float32::ConstPtr& msg) {
    /*
        Rear flipper commands callback function.  This function is called every time
        commands are received over the rear flipper control topic.
    */
    double flipper_vel = msg->data;
    // PWM Control (copied from origin drrobot_player node code)
    int flipperPWM = -flipper_motor_direction * flipper_vel * 16384 + 16384;
    if (flipperPWM > 32767) flipperPWM = 32767;
    if (flipperPWM < 0) flipperPWM = 0;
    // send PWM commands to flipper motors
    // TODO: switch this from all command to just single command
    jaguar_driver->sendMotorCtrlAllCmd(PWM, NOCONTROL, flipperPWM, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL);
}

void JaguarPlayer::headlightCallback(const std_msgs::Bool::ConstPtr& msg) {
    /*
        Headlight commands callback function.  This function is called every time 
        commands are received over the headlight control topic.
    */
    if (msg->data)
        jaguar_driver->setCustomIO(0x80);
    else
        jaguar_driver->setCustomIO(0x7f);
}

void JaguarPlayer::update() {
    /*
        This function is called by the run function at each time step.
        Update is responsible for updating all sensor data received from the motorola eval board.
    */
    if (jaguar_driver->portOpen()) {
        // Get Motor sensor information
        jaguar_driver->readMotorSensorData(&motor_sensor_data);
        ros::Time motor_sensor_time = ros::Time::now();
        // Get Motion board sensor information
        jaguar_driver->readStandardSensorData(&standard_sensor_data);
        ros::Time motion_board_time = ros::Time::now();
        // Get custom sensor information
        //  - Documented in jaguar_ros/docs/jaguar_customADSensor.txt
        jaguar_driver->readCustomSensorData(&custom_sensor_data);
        ros::Time custom_sensor_time = ros::Time::now();

        ///////////////////////////////////////////////////////////////////
        // Motor sensor numbers: (looking towards front of robot)
        // - 0: front flipper
        // - 1: Rear flipper
        // - 2: not used
        // - 3: right(port) drive track
        // - 4: left(starboard) drive track
        // - 5: not used
        // TODO: MAKE array accesses robust (try, catch)
        // Build Jaguar motor sensor data message (contains encoder data for all motors)
        jaguar_ros::JaguarMotorSensorData motor_sensor_msg;
        jaguar_ros::MotorInfo front_flipper_info;
        jaguar_ros::MotorInfo rear_flipper_info;
        jaguar_ros::MotorInfo port_drive_info;
        jaguar_ros::MotorInfo starboard_drive_info;
        motor_sensor_msg.header.stamp = motor_sensor_time;
        // - Build front flipper message
        front_flipper_info.header.stamp = motor_sensor_time;
        front_flipper_info.encoder_pos = motor_sensor_data.motorSensorEncoderPos[motor_ids["FRONT_FLIPPER"]];
        front_flipper_info.encoder_vel = motor_sensor_data.motorSensorEncoderVel[motor_ids["FRONT_FLIPPER"]];
        front_flipper_info.encoder_dir = motor_sensor_data.motorSensorEncoderDir[motor_ids["FRONT_FLIPPER"]];
        front_flipper_info.motor_pwm = motor_sensor_data.motorSensorPWM[motor_ids["FRONT_FLIPPER"]];
        // - Build rear flipper message
        rear_flipper_info.header.stamp = motor_sensor_time;
        rear_flipper_info.encoder_pos = motor_sensor_data.motorSensorEncoderPos[motor_ids["REAR_FLIPPER"]];
        rear_flipper_info.encoder_vel = motor_sensor_data.motorSensorEncoderVel[motor_ids["REAR_FLIPPER"]];
        rear_flipper_info.encoder_dir = motor_sensor_data.motorSensorEncoderDir[motor_ids["REAR_FLIPPER"]];
        rear_flipper_info.motor_pwm = motor_sensor_data.motorSensorPWM[motor_ids["REAR_FLIPPER"]];
        //  - Build port drive message
        port_drive_info.header.stamp = motor_sensor_time;
        port_drive_info.encoder_pos = motor_sensor_data.motorSensorEncoderPos[motor_ids["PORT_DRIVE"]];
        port_drive_info.encoder_vel = motor_sensor_data.motorSensorEncoderVel[motor_ids["PORT_DRIVE"]];
        port_drive_info.encoder_dir = motor_sensor_data.motorSensorEncoderDir[motor_ids["PORT_DRIVE"]];
        port_drive_info.motor_pwm = motor_sensor_data.motorSensorPWM[motor_ids["PORT_DRIVE"]];
        //  - Build starboard drive message
        starboard_drive_info.header.stamp = motor_sensor_time;
        starboard_drive_info.encoder_pos = motor_sensor_data.motorSensorEncoderPos[motor_ids["STARBOARD_DRIVE"]];
        starboard_drive_info.encoder_vel = motor_sensor_data.motorSensorEncoderVel[motor_ids["STARBOARD_DRIVE"]];
        starboard_drive_info.encoder_dir = motor_sensor_data.motorSensorEncoderDir[motor_ids["STARBOARD_DRIVE"]];
        starboard_drive_info.motor_pwm = motor_sensor_data.motorSensorPWM[motor_ids["STARBOARD_DRIVE"]];
        // Add motor info msgs to motor_sensor_msg
        motor_sensor_msg.front_flipper = front_flipper_info;
        motor_sensor_msg.rear_flipper = rear_flipper_info;
        motor_sensor_msg.port_drive = port_drive_info;
        motor_sensor_msg.starboard_drive = starboard_drive_info;
        // Publish motor info message
        motor_info_pub.publish(motor_sensor_msg);
        ///////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // Build and publish message that contains motion board sensors.
        jaguar_ros::JaguarMotionBoardInfo motion_board_info_msg;
        motion_board_info_msg.header.stamp = motion_board_time;

        motion_board_info_msg.heat_sensors.resize(motion_board_heat_sensor_cnt);
        for (int i = 0; i < motion_board_heat_sensor_cnt; i++) {
            // convert heat sensor raw reading to celsius (math is documented in WiRobot SDK API Reference Manual for drrobot Jaguar)
            int ival = standard_sensor_data.overHeatSensorData[i];
            motion_board_info_msg.heat_sensors[i] = ival; // Just put raw values in; not sure if drrobot's documented formulas are correct (below).
            //motion_board_info_msg.heat_sensors[i] = 100 - ((ival - 980) / 11.6);
            //motion_board_info_msg.heat_sensors[i] = (ival - 1256) / 34.8;
        }
        motion_board_info_msg.board_power_vol = (double)standard_sensor_data.boardPowerVol * 9.0 / 4095.0;  // Data comes in raw (0 - 4095), convert to voltage (9volt max) (copied from original drrobot_player)
        motion_board_info_msg.board_ref_vol = (double)standard_sensor_data.refVol / 4095.0 * 6.0;           // Convert to voltage (copied from original drrobot_player)
        motion_board_info_msg.motor_power_vol = (double)standard_sensor_data.motorPowerVol * 34.498 / 4095.0;   // Convert to voltage (copied from original drrobot_player)
        // publish motion board info message
        motion_board_info_pub.publish(motion_board_info_msg);
        ///////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // Extract relevant information from customADSensor data
        ///////////////////////////////////////////////////////////////////
        // AD0- board 5V voltage
        // AD1 – motor power(battery power) voltage
        // AD2 – left rear motor temperature
        // AD3 - right rear motor temperature
        // AD4 – left front motor temperature
        // AD5 – front flip motor temperature
        // AD6 – right front motor temperature
        // AD7 – rear flip motor temperature
        // get battery voltage and convert from raw to voltage
        double battery_voltage = ((double)custom_sensor_data.customADData[1]) / 4095 * 34.498;
        // get motor temperatures and convert from raw to celsius
        double left_rear_drive_motor_temp = drrobotRawMotorTemperatureToCelsius((double)custom_sensor_data.customADData[2]);
        double right_rear_drive_motor_temp = drrobotRawMotorTemperatureToCelsius((double)custom_sensor_data.customADData[3]);
        double left_front_drive_motor_temp = drrobotRawMotorTemperatureToCelsius((double)custom_sensor_data.customADData[4]);
        double front_flipper_motor_temp = drrobotRawMotorTemperatureToCelsius((double)custom_sensor_data.customADData[5]);
        double right_front_drive_motor_temp = drrobotRawMotorTemperatureToCelsius((double)custom_sensor_data.customADData[6]);
        double rear_flipper_motor_temp = drrobotRawMotorTemperatureToCelsius((double)custom_sensor_data.customADData[7]);
        ///////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // Publish Battery voltage 
        jaguar_ros::JaguarBatteryInfo battery_info_msg;
        battery_info_msg.header.stamp = custom_sensor_time;
        battery_info_msg.voltage = battery_voltage;
        // publish battery info
        battery_info_pub.publish(battery_info_msg);
        ///////////////////////////////////////////////////////////////////
        
        ///////////////////////////////////////////////////////////////////
        // Publish Motor temperatures
        jaguar_ros::JaguarMotorTemps motor_temps_msg;
        motor_temps_msg.header.stamp = custom_sensor_time;
        motor_temps_msg.front_port_drive_temp = right_front_drive_motor_temp;
        motor_temps_msg.front_starboard_drive_temp = left_front_drive_motor_temp;
        motor_temps_msg.rear_port_drive_temp = right_rear_drive_motor_temp;
        motor_temps_msg.rear_starboard_drive_temp = left_rear_drive_motor_temp;
        motor_temps_msg.front_flipper_temp = front_flipper_motor_temp;
        motor_temps_msg.rear_flipper_temp = rear_flipper_motor_temp;
        motor_temps_pub.publish(motor_temps_msg);
        ///////////////////////////////////////////////////////////////////
    }   
}

void JaguarPlayer::run() {
    /*
      Main Jaguar Player run loop.
    */
    ros::Rate rate(10); // Create target rate for main loop to run at
    while (ros::ok()) {
        update();
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {

    // Initialize ROS
    ros::init(argc, argv, "jaguar_player");
    // Create jaguar_player node
    JaguarPlayer jag_player;
    // Run jaguar_player node
    jag_player.run();
    return 0;
}
