#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "DrRobotMotionSensorDriver.hpp"

using std::string;

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
// Motor Constants
//   - Drive motor constants
int DEFAULT_DRIVE_MOTOR_DIRECTION = 1;
double DEFAULT_DRIVE_MOTOR_MAX_SPEED = 1.0;
double DEFAULT_DRIVE_MOTOR_MIN_SPEED = 0.1;
//  - Flipper motor constants
int DEFAULT_FLIPPER_MOTOR_DIRECTION = 1;
double DEFAULT_FLIPPER_MOTOR_MAX_SPEED = 1.0;
double DEFAULT_FLIPPER_MOTOR_MIN_SPEED = 0.1;

////////////////////////////////////////////////////////

class JaguarPlayer {
    /*
        This class defines the ROS node that interfaces with the DrRobot Jaguar's
        motorolla evaluation board.  This board controls robot PWM control, motor encoder data reportings,
        the front headlights, and <OTHER SENSORS TO BE DETERMINED>
    */
public:
    JaguarPlayer();         // Constuctor
    ~JaguarPlayer();        // Destructor
    
    void run(void);         // Main run function for jaguar player node

private:
    
    // Jaguar Driver
    DrRobotMotionSensorDriver* jaguar_driver;

    ros::NodeHandle node_handle;

    ros::Subscriber drive_vel_sub;
    ros::Subscriber front_flipper_cmds_sub;
    ros::Subscriber rear_flipper_cmds_sub;
    ros::Subscriber headlight_cmds_sub;

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
    // Motor parameters (can't use floats because of ros param server compatibility issue)
    double drive_max_speed;
    double drive_min_speed;
    int drive_motor_direction;
    double flipper_max_speed;
    double flipper_min_speed;
    int flipper_motor_direction;

    void connect(void);

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
    // - Load Motor parameters
    //   - Drive
    node_handle.param<double>("motors/drive/max_speed", drive_max_speed, DEFAULT_DRIVE_MOTOR_MAX_SPEED);
    node_handle.param<double>("motors/drive/min_speed", drive_min_speed, DEFAULT_DRIVE_MOTOR_MIN_SPEED);
    node_handle.param<int>("motors/drive/motor_direction", drive_motor_direction, DEFAULT_DRIVE_MOTOR_DIRECTION);
    //   - Flippers
    node_handle.param<double>("motors/flippers/max_speed", flipper_max_speed, DEFAULT_FLIPPER_MOTOR_MAX_SPEED);
    node_handle.param<double>("motors/flippers/min_speed", flipper_min_speed, DEFAULT_FLIPPER_MOTOR_MIN_SPEED);
    node_handle.param<int>("motors/flippers/motor_direction", flipper_motor_direction, DEFAULT_FLIPPER_MOTOR_DIRECTION);
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
    // - Set jaguar player IP (IP of motorolla eval board)
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
    /**/
    ROS_INFO("FRONT FLIPPER COMMAND RECEIVED");
}

void JaguarPlayer::rearFlipperCallback(const std_msgs::Float32::ConstPtr& msg) {
    /**/
    ROS_INFO("REAR FLIPPER COMMAND RECEIVED");
}

void JaguarPlayer::headlightCallback(const std_msgs::Bool::ConstPtr& msg) {
    /**/
    ROS_INFO("HEADLIGHT COMMAND RECEIVED");
}

void JaguarPlayer::run() {
    /*
      Main Jaguar Player run loop.
    */

    ros::Rate rate(100); // Create target rate for main loop to run at
    while (ros::ok()) {
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