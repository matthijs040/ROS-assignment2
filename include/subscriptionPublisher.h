#ifndef SUBSCRIPTIONPUBLISHER_H
#define SUBSCRIPTIONPUBLISHER_H

#include "ros/ros.h"
#include <functional>
#include <string>

template<typename pubtype, typename subtype>
class SubscriptionPublisher
{
public:

  // NOTE: function can be null and no manipulation will take place on subtype.
  SubscriptionPublisher(std::string pubtopic, std::string subtopic, std::function<pubtype(subtype)> function)
  {
    pub_ = n_.advertise<pubtype>(pubtopic, 1);
    sub_ = n_.subscribe(subtopic, 1, &SubscriptionPublisher::callback, this);
    if(function)
    { _function = function; }

  }

  // NOTE: only produces a publish if a valid function is given or subtype == pubtype.
  void callback(const subtype& in)
  {
    if(_function)
    {
      pub_.publish( _function(in) );
    }
    else if( std::is_same<pubtype, subtype>::value )
    {
      pub_.publish(in);
    }   
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

  std::function< pubtype(subtype) > _function;

};

#endif //SUBSCRIPTIONPUBLISHER_H