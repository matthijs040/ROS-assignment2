#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include "../include/pointshoot.h"
#include <exception>
#include "angles/angles.h"
#include "tf/tf.h"

 geometry_msgs::Twist Pointshoot::updatePath(const nav_msgs::Odometry odomUpdate)
    {
        switch (state)
        {
            
        case State::POINT_START :
        {
            const double currentAngle = tf::getYaw( odomUpdate.pose.pose.orientation );


            // If the angle is not yet calculated, calculate it once. It is reset once the goal is reached. To be recalculated on the next set goal.
            if(!angle)
            { 
                double dx = odomUpdate.pose.pose.position.x - goal.pose.position.x;
                double dz = odomUpdate.pose.pose.position.z - goal.pose.position.z;
                //  double dz = odomUpdate.pose.pose.position.z - goal.pose.position.z;

                angle =  atan2(dz, dx) ;
                std::cout << "calculated the angle: " << std::to_string( angles::to_degrees(angle.value()) ) << '\n'; 
                std::cout << "And current angle == " << std::to_string( angles::to_degrees(currentAngle) )  << '\n'; 
            
            }

            const double desiredAngle = angle.value();

            if( currentAngle < desiredAngle + 0.0001 && desiredAngle > currentAngle - 0.0001 )
            {
                // Reset starting angle. To be recalculated when a new goal is set.
                angle.reset();
                state = State::SHOOT;
                std::cout << "completed POINT_START \n";
                return geometry_msgs::Twist();
            }
            else
            {
                geometry_msgs::Twist move = geometry_msgs::Twist();
                double error =  desiredAngle - currentAngle;
                move.angular.z = angularP *  error ;
                std::cout << "moving at X: " + std::to_string( angles::to_degrees(currentAngle) ) + " \n";

                // TODO: Threshold movement?

                return move;                
            }
            
        }
        break;

        case State::SHOOT :
        {
            const double cX = odomUpdate.pose.pose.position.x, cY = odomUpdate.pose.pose.position.y, cZ = odomUpdate.pose.pose.position.z,
                         gX = goal.pose.orientation.x,         gY = goal.pose.orientation.y,         gZ = goal.pose.orientation.z;


            if(cX == gX && cY == gY && cZ == gZ )
            {
                state = State::POINT_FINISH;
            }
            else
            {
                std::cout << "currently in shoot! \n";

                geometry_msgs::Twist move = geometry_msgs::Twist();
                const double errX = gX - cX;
                const double errY = gY - cY;
                const double errZ = gZ - cZ;
                
                move.linear.x = linearP * errX;
                move.linear.y = linearP * errY;
                move.linear.z = linearP * errZ;

                // TODO: Threshold movement?

                return move;                
            }
        }
        break;

        case State::POINT_FINISH:
        {
            if(odomUpdate.pose.pose.orientation.x == goal.pose.orientation.x 
            && odomUpdate.pose.pose.orientation.y == goal.pose.orientation.y 
            && odomUpdate.pose.pose.orientation.z == goal.pose.orientation.z )
            {
                state = State::DONE;
                std::cout << "completed POINT_START \n";
            }
            else
            {
                geometry_msgs::Twist move = geometry_msgs::Twist();
                move.angular.x = angularP * odomUpdate.pose.pose.orientation.x - goal.pose.orientation.x;
                move.angular.y = angularP * odomUpdate.pose.pose.orientation.y - goal.pose.orientation.y;
                move.angular.z = angularP * odomUpdate.pose.pose.orientation.z - goal.pose.orientation.z;

                // TODO: Threshold movement?

                return move;
            }           
        }
        break;

        case State::DONE :
        {
            //NOTE: State is reset to START when a new goal is set through setGoal()
            return geometry_msgs::Twist();           
        } 
        break;
        
        default:
        {
            //throw std::logic_error("Invalid state in updatepath");
            std::cout << "REACHING DEFAULT!!! \n";
            state = State::POINT_START;
        }
                 
        } // switch (state)
        
        return geometry_msgs::Twist();  
    } 