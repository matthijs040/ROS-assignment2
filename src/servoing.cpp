#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include "../include/servoing.h"
#include <exception>
#include "angles/angles.h"
#include "tf/tf.h"

geometry_msgs::Twist Servoing::updatePath(const nav_msgs::Odometry odomUpdate)
{
    geometry_msgs::Twist move = geometry_msgs::Twist();

    if(!Servoing::angFinished)
    {

    const double currentAngle = tf::getYaw( odomUpdate.pose.pose.orientation );
    const double desiredAngle = tf::getYaw( goal.pose.orientation );


    if( currentAngle < desiredAngle + 0.0001 && currentAngle > desiredAngle  - 0.0001 )
    {
        // Reset starting angle. To be recalculated when a new goal is set.
        angFinished = true;
        std::cout << "completed POINT_START \n";
        return geometry_msgs::Twist();
    }
    else
    {

        double deviation =  desiredAngle - currentAngle;
        move.angular.z = angularP *  deviation ;
        std::cout << "moving at X: " + std::to_string( angles::to_degrees(currentAngle) ) + " \n";
        
        // TODO: Threshold movement?
                      
    }
    }
    if(!Servoing::linFinished)
    {
         const double cX = odomUpdate.pose.pose.position.x, cY = odomUpdate.pose.pose.position.y, cZ = odomUpdate.pose.pose.position.z,
                             gX = goal.pose.position.x,         gY = goal.pose.position.y,         gZ = goal.pose.position.z;


        //currentAngle < desiredAngle + 0.0001 && currentAngle > desiredAngle  - 0.0001
        if(  cX < gX + 0.01 && cX > gX - 0.01 
          && cY < gY + 0.01 && cY > gY - 0.01 
          && cZ < gZ + 0.01 && cZ > gZ - 0.01 )
        {
            linFinished = true;
            std::cout << "completed SHOOT \n";
        }
        else
        {
            //std::cout << "currently in shoot! \n";

            const double deviationX = gX - cX;
            const double deviationY = gY - cY;
            const double deviationZ = gZ - cZ;

            move.linear.x = abs(linearP * deviationX);
            move.linear.y = abs(linearP * deviationY);
            move.linear.z = abs(linearP * deviationZ);

            // TODO: Threshold movement?

        }
    }

    return move;
}