#include <ros/ros.h>

#include <stdio.h>
#include <string>

#include <DrRobotMotionSensorDriver.hpp>

using std::string;

using namespace DrRobot_MotionSensorDriver;

////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////
// Networking
string DEFAULT_JAGUAR_IP = "192.168.0.70";
string DEFAULT_JAGUAR_PORT = 10001;
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
    void run(void);         // Main run function for jaguar player node

private:
    
    // Jaguar Driver
    DrRobotMotionSensorDriver* drrobot_driver;

    ros::NodeHandle node_handle;

    ros::Subscriber drive_vel_sub;

    // Networking variables
    string jaguar_ip;
    int jaguar_port;
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


};

JaguarPlayer::JaguarPlayer() {
    /*
        Jaguar Player default contructor.
    */

    // Load Networking information
    node_handle.param<string>("player/ip", jaguar_ip, DEFAULT_JAGUAR_IP);
    node_handle.param<string>("player/port", jaguar_port, DEFAULT_JAGUAR_PORT);

    // Load topic names
    node_handle.param<string>("robot_control_topics/drive_control", drive_vel_topic, DEFAULT_DRIVE_VEL_TOPIC);

    // Load Motor parameters
    // TODO (AS NEEDED)

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