#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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
string DEFAULT_JAGUAR_IP = "192.168.0.70";
int DEFAULT_JAGUAR_NETWORK_PORT = 10001;
string DEFAULT_JAGUAR_SERIAL_PORT = "/dev/ttyS0";
CommMethod DEFAULT_COMM_METHOD = Network;       // Default comm method if invalid comm method parameter is given.
string DEFAULT_COMM_METHOD_PARAM = "network";    // Default comm method if parameter is unavailable
// Topics
string DEFAULT_DRIVE_VEL_TOPIC = "cmd_vel";
// Motor Constants
int DEFAULT_MOTOR_DIRECTION = 1;
int DEFAULT_ENCODER_CIRCLE_CNT = 800;
double DEFAULT_WHEEL_RADIUS = 0.0835;   // in meters
double DEFAULT_WHEEL_DISTANCE = 0.305;  // in meters
double DEFAULT_MIN_SPEED = 0.1;



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

    // Networking variables
    string comm_method;
    string jaguar_ip;
    int jaguar_network_port;
    string jaguar_serial_port;
    struct DrRobotMotionConfig jaguar_driver_config;
    // Topic names
    string drive_vel_topic;
    // Motor parameters
    int encoder_circle_cnt;
    int motor_direction;
    double wheel_distance;
    double wheel_radius;
    double min_speed;
    double max_speed;

    void connect(void);

    void driveVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

};

JaguarPlayer::JaguarPlayer() {
    /*
        Jaguar Player default contructor.
    */

    ///////////////////////////////////////////////////////
    // Load parameters from parameter server
    ///////////////////////////////////////////////////////
    // Load Networking information
    node_handle.param<string>("player/ip", jaguar_ip, DEFAULT_JAGUAR_IP);
    node_handle.param<int>("player/port", jaguar_network_port, DEFAULT_JAGUAR_NETWORK_PORT);
    node_handle.param<string>("player/serial_port", jaguar_serial_port, DEFAULT_JAGUAR_SERIAL_PORT);
    node_handle.param<string>("player/comm_method", comm_method, DEFAULT_COMM_METHOD_PARAM);
    // Load topic names
    node_handle.param<string>("robot_control_topics/drive_control", drive_vel_topic, DEFAULT_DRIVE_VEL_TOPIC);

    // Load Motor parameters
    // TODO (AS NEEDED)
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // Setup Jaguar driver configuration
    ///////////////////////////////////////////////////////
    //  - Set communication type (network or serial)
    if (strcasecmp(comm_method.c_str(), "network") == 0) {
        // use network comms
        jaguar_driver_config.commMethod = Network;
    } else if (strcasecmp(comm_method.c_str(), "serial") == 0) {
        // use serial comms
        jaguar_driver_config.commMethod = Serial;
    } else {
        // use default comms
        jaguar_driver_config.commMethod = DEFAULT_COMM_METHOD;
    }
    // - Set robot type (Jaguar)
    jaguar_driver_config.boardType = Jaguar;
    // - Set jaguar player network port number
    jaguar_driver_config.portNum = jaguar_network_port;
    // - Set jaguar player IP (IP of motorolla eval board)
    strcpy(jaguar_driver_config.robotIP, jaguar_ip.c_str());
    // - Set jaguar player serial port 
    strcpy(jaguar_driver_config.serialPortName, jaguar_serial_port.c_str());
    ///////////////////////////////////////////////////////
    
    ///////////////////////////////////////////////////////
    // Create Jaguar Driver
    ///////////////////////////////////////////////////////
    jaguar_driver = new DrRobotMotionSensorDriver();
    jaguar_driver->setDrRobotMotionDriverConfig(&jaguar_driver_config);

    ///////////////////////////////////////////////////////
    // Robot startup
    ///////////////////////////////////////////////////////    
    // connect to robot
    ROS_INFO("ENTERING CONNECT FUNCTION");
    connect();
    ROS_INFO("OUT OF CONNECT FUNCTION");

    ///////////////////////////////////////////////////////
    // Setup ROS Publishers
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // Setup ROS Subscribers
    ///////////////////////////////////////////////////////
    drive_vel_sub = node_handle.subscribe<geometry_msgs::Twist>(drive_vel_topic, 1000, boost::bind(&JaguarPlayer::driveVelCallback, this, _1));


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
     *  This function attempts to connect to Jaguar robot.
     *  If it fails, it retries.
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
     * Gets called when message is received over the drive control topic (typically cmd_vel).
    */
     double lin_vel = msg->linear.x;
     double rot_vel = msg->angular.z;
     /* PWM control
     int linPWM = -motor_direction * lin_vel * 16384 + 16384;
     int rotPWM = -motor_direction * rot_vel * 16384 + 16384;
     if (linPWM > 32767) linPWM = 32767;
     if (linPWM < 0) linPWM = 0;
     if (rotPWM > 32767) rotPWM = 32767;
     if (rotPWM < 0) rotPWM = 0;

     jaguar_driver->sendMotorCtrlAllCmd(PWM, NOCONTROL, NOCONTROL, NOCONTROL, linPWM, rotPWM, NOCONTROL);
    */
     // Velocity control
     jaguar_driver->sendMotorCtrlAllCmd(Velocity, NOCONTROL, NOCONTROL, NOCONTROL, lin_vel, rot_vel, NOCONTROL);
}

void JaguarPlayer::run() {

    ros::Rate rate(10); // Create target rate for main loop to run at
    while (ros::ok()) {

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