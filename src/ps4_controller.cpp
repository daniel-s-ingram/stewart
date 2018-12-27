#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class Controller
{
    private: ros::Publisher pub;
    private: geometry_msgs::Twist twist_msg;

    public: Controller(int argc, char **argv)
    {
        ros::init(argc, argv, "ps4_controller");
        ros::NodeHandle nh;
        pub = nh.advertise<geometry_msgs::Twist>("/stewart/platform_twist", 100);
        ros::Subscriber sub = nh.subscribe("/joy", 100, &Controller::callback, this);
        main();
    }

    private: void main(void)
    {
        ros::spin();
    }

    private: void callback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        twist_msg.linear.x = -msg->axes[0];
        twist_msg.linear.y = msg->axes[1];
        twist_msg.angular.x = -msg->axes[4];
        twist_msg.angular.y = -msg->axes[3];
        if (msg->buttons[7] && twist_msg.linear.z < 1)
        {
            twist_msg.linear.z += 0.01;
        }
        else if (msg->buttons[6] && twist_msg.linear.z > 0)
        {
            twist_msg.linear.z -= 0.01;
        }
        if (msg->buttons[5] && twist_msg.angular.z < M_PI/2.0)
        {
            twist_msg.angular.z += 0.01;
        }
        else if (msg->buttons[4] && twist_msg.angular.z > -M_PI/2.0)
        {
            twist_msg.angular.z -= 0.01;
        }
        pub.publish(twist_msg);
    }

};

int main(int argc, char **argv)
{
    Controller controller(argc, argv);
}