#include "ros/ros.h"
#include "controller_msgs/MotorSignal.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <inttypes.h>
#include <cmath>

using namespace std;

ros::Publisher pub;

float maxMotorOutput;

class math
{
public:
    template <class T>
    static constexpr const T &clamp(const T &v, const T &lo, const T &hi)
    {
        assert(!(hi < lo));
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
};

void twistReceived(const geometry_msgs::Twist::ConstPtr &twist)
{
    controller_msgs::MotorSignal motor;
    
    double rotation = twist->angular.z;
    motor.leftMotor = (rotation > 0) ? -1 : 1;
    motor.rightMotor = (rotation > 0) ? 1 : -1;

    motor.leftMotor *= abs(rotation);
    motor.rightMotor *= abs(rotation);

    double linear = twist->linear.x;
    motor.leftMotor += linear;
    motor.rightMotor += linear;

    double max = std::max(abs(motor.leftMotor), abs(motor.rightMotor));

    if (max > maxMotorOutput) {
        motor.leftMotor /= max / maxMotorOutput;
        motor.rightMotor /= max / maxMotorOutput;
    }

    if (abs(motor.leftMotor) <= 0.01) motor.leftMotor = 0;
    if (abs(motor.rightMotor) <= 0.01) motor.rightMotor = 0;

    pub.publish(motor);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_to_motorsignal");
    ros::NodeHandle nodeHandle;

    pub = nodeHandle.advertise<controller_msgs::MotorSignal>("signal", 5);

    ros::param::param("~max_motor_signal", maxMotorOutput, 1.0f);

    ros::Subscriber sub = nodeHandle.subscribe("twist", 1, twistReceived);

    ros::spin();

    return 0;
}
