#ifndef SERVOING_H
#define SERVOING_H

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <optional>

class Servoing 
{
    private:
    float angularP = 1.2;
    float linearP = 1.2;
    
    geometry_msgs::PoseStamped goal;

    public:

    bool angFinished, linFinished;

    // Create a Servoing object that, when polled for a new command with updated telemetry will provide the right command to reach the destination.
    // The end condition of which would be a Twist message with no motion.
    Servoing(geometry_msgs::PoseStamped goal)
    {
        this->goal = goal;
        angFinished = false;
        linFinished = false;
    }

    // Override the goal with a new goal position + orientation.
    void setGoal(geometry_msgs::PoseStamped goal)
    {
        this->goal = goal;
        angFinished = false;
        linFinished = false;
    }

    // Possible function when making the goal a vector of goals? Making it set the newer goal after reaching its current goal.
    // void queueGoal(geometry_msgs::PoseStamped::ConstPtr& goal);

    // Calculates the point-shoot twist required to make a robot move towards goal.
    // This calculation requires the most recent odom update.
    geometry_msgs::Twist updatePath(const nav_msgs::Odometry odomUpdate);

}; // class Servoing

//Servoing_H
#endif