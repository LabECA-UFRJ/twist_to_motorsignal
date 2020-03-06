#include "ros/ros.h"
#include "controller_msgs/MotorSignal.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <inttypes.h>

#include <cmath>

using namespace std;

ros::Publisher pub;

void twistReceived(const geometry_msgs::Twist::ConstPtr &twist)
{
    double rotation = twist->angular.z;

    controller_msgs::MotorSignal motor;
    motor.leftMotor = (rotation > 0) ? -1 : 1;
    motor.rightMotor = (rotation > 0) ? 1 : -1;

    if (abs(rotation) <= 0.001) {
        motor.leftMotor = 0;
        motor.rightMotor = 0;
    }

    pub.publish(motor);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_to_motorsignal");
    ros::NodeHandle nodeHandle;

    pub = nodeHandle.advertise<controller_msgs::MotorSignal>("signal", 5);

    ros::Subscriber sub = nodeHandle.subscribe("twist", 1, twistReceived);

    ros::spin();

    return 0;
}
