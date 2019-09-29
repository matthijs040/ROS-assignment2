#include "ros/ros.h"

#include "../include/servoing.h"
#include "../include/subscriptionPublisher.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"

#include <string>


std::unique_ptr<Servoing> controller;

void goalCallback(geometry_msgs::PoseStampedPtr msg)
{
    if(controller)
    {
        controller->setGoal(*msg);
    }
    else
    {
        controller = std::make_unique<Servoing>(*msg);       
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
    ros::init(argc, argv, "servoing_node");
    ros::NodeHandle n;

    ros::Subscriber subscriber = n.subscribe("/goal", 10, goalCallback);

    SubscriptionPublisher<geometry_msgs::Twist, nav_msgs::OdometryPtr> subpub("/cmd_vel", "/odom", getpath);

    std::cout << "awaiting a poseStamped publish on the /goal topic. \n";

    ros::spin();

    return 0;
}
