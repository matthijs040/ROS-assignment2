#ifndef POINTSHOOT_H
#define POINTSHOOT_H

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <optional>

class Pointshoot 
{
    private:
    float angularP = 1.2;
    float linearP = 1.2;
    
    std::optional<double> angle;

    float length;

    geometry_msgs::PoseStamped goal;

    geometry_msgs::Twist twistUpdate;
    public:

    enum class State
    {
        POINT_START,
        SHOOT,
        POINT_FINISH,
        DONE
    }state;

    // Create a Pointshoot object that, when polled for a new command with updated telemetry will provide the right command to reach the destination.
    // The end condition of which would be a Twist message with no motion.
    Pointshoot(geometry_msgs::PoseStamped goal)
    {
        this->goal = goal;
        state = State::POINT_START;
    }

    // Override the goal with a new goal position + orientation.
    void setGoal(geometry_msgs::PoseStamped goal)
    {
        this->goal = goal;
        state = State::POINT_START;
    }

    // Possible function when making the goal a vector of goals? Making it set the newer goal after reaching its current goal.
    // void queueGoal(geometry_msgs::PoseStamped::ConstPtr& goal);

    // Calculates the point-shoot twist required to make a robot move towards goal.
    // This calculation requires the most recent odom update.
    geometry_msgs::Twist updatePath(const nav_msgs::Odometry odomUpdate);

}; // class pointshoot

//POINTSHOOT_H
#endif