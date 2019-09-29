#include "ros/ros.h"

#include "../include/pointshoot.h"
#include "../include/subscriptionPublisher.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"

#include <functional>
#include <string>


std::unique_ptr<Pointshoot> controller;

void goalCallback(geometry_msgs::PoseStampedPtr msg)
{
    if(controller)
    {
        controller->setGoal(*msg);
    }
    else
    {
        controller = std::make_unique<Pointshoot>(*msg);       
    }
}

geometry_msgs::Twist getpath(nav_msgs::OdometryPtr msg)
{
    if(controller)
    { 
        geometry_msgs::Twist path = controller->updatePath(*msg); 
        return path;
    }

    return geometry_msgs::Twist();
}

int main(int argc, char *argv[])
{
    //Startup node.
    ros::init(argc, argv, "pointshoot_node");
    ros::NodeHandle n;

    ros::Subscriber subscriber = n.subscribe("/goal", 10, goalCallback);

    SubscriptionPublisher<geometry_msgs::Twist, nav_msgs::OdometryPtr> subpub("/cmd_vel", "/odom", getpath);

    ros::spin();

    return 0;
}
