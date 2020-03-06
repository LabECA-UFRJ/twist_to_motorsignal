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
    double rotation = twist->angular.z;

    controller_msgs::MotorSignal motor;
    motor.leftMotor = (rotation > 0) ? -1 : 1;
    motor.rightMotor = (rotation > 0) ? 1 : -1;

    if (abs(rotation) <= 0.001)
    {
        motor.leftMotor = 0;
        motor.rightMotor = 0;
    }

    motor.leftMotor = math::clamp(motor.leftMotor, -maxMotorOutput, maxMotorOutput);
    motor.rightMotor = math::clamp(motor.rightMotor, -maxMotorOutput, maxMotorOutput);

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
