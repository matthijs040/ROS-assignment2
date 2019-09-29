#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/pointshoot.h"
#include <exception>
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
                double dx = goal.pose.position.x - odomUpdate.pose.pose.position.x;
                double dy = goal.pose.position.y - odomUpdate.pose.pose.position.y;
                //  double dz = odomUpdate.pose.pose.position.z - goal.pose.position.z;

                angle =  atan2(dy, dx) ;
                //std::cout << "calculated the angle: " << std::to_string( angles::to_degrees(angle.value()) ) << '\n'; 
                //std::cout << "And current angle == " << std::to_string( angles::to_degrees(currentAngle) )  << '\n'; 
            
            }

            const double desiredAngle = angle.value();

            if( currentAngle < desiredAngle + 0.0001 && currentAngle > desiredAngle  - 0.0001 )
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
                double deviation =  desiredAngle - currentAngle;
                move.angular.z = angularP *  deviation ;
                //std::cout << "moving at X: " + std::to_string( angles::to_degrees(currentAngle) ) + " \n";

                // TODO: Threshold movement?

                return move;                
            }
            
        }
        break;

        case State::SHOOT :
        {
            const double cX = odomUpdate.pose.pose.position.x, cY = odomUpdate.pose.pose.position.y, cZ = odomUpdate.pose.pose.position.z,
                         gX = goal.pose.position.x,         gY = goal.pose.position.y,         gZ = goal.pose.position.z;


            //currentAngle < desiredAngle + 0.0001 && currentAngle > desiredAngle  - 0.0001
            if(  cX < gX + 0.01 && cX > gX - 0.01 
              && cY < gY + 0.01 && cY > gY - 0.01 
              && cZ < gZ + 0.01 && cZ > gZ - 0.01 )
            {
                state = State::POINT_FINISH;
                std::cout << "completed SHOOT \n";
                return geometry_msgs::Twist();
            }
            else
            {
                //std::cout << "currently in shoot! \n";

                geometry_msgs::Twist move = geometry_msgs::Twist();
                const double deviationX = gX - cX;
                const double deviationY = gY - cY;
                const double deviationZ = gZ - cZ;
                
                move.linear.x = abs(linearP * deviationX);
                move.linear.y = abs(linearP * deviationY);
                move.linear.z = abs(linearP * deviationZ);

                // TODO: Threshold movement?

                return move;                
            }
        }
        break;

        case State::POINT_FINISH:
        {
             const double currentAngle = tf::getYaw( odomUpdate.pose.pose.orientation );
             const double desiredAngle = goal.pose.orientation.z;

            if( currentAngle < desiredAngle + 0.0001 && currentAngle > desiredAngle  - 0.0001 )
            {
                state = State::DONE;
                std::cout << "completed POINT_FINISH \n";
            }
            else
            {
                geometry_msgs::Twist move = geometry_msgs::Twist();
                const double deviation = desiredAngle - currentAngle;
                move.angular.z = angularP * deviation ;

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
            throw std::logic_error("Invalid state in updatepath");
        }
                 
        } // switch (state)
        // throw std::logic_error("Invalid state in updatepath");
    } 