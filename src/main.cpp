#include "controller_msgs/MotorSignal.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include <cmath>
#include <inttypes.h>
#include <iostream>

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

    double u = twist->linear.x;
    double w = twist->angular.z;

    double r = 0.25;
    double r_inv = 1 / r;
    double L = 0.5;

    motor.leftMotor = r_inv * (u + 0.5 * L * w);
    motor.rightMotor = r_inv * (u - 0.5 * L * w);

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
