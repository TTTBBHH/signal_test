#ifndef  VIEW_SUBSCRIBER_HPP_
#define VIEW_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<mutex>

class Viewsubscriber {
  public:
    Viewsubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    Viewsubscriber() = default;
  
  public:
    geometry_msgs::Twist signal_test;

  private:
    void msg_callback(const geometry_msgs::Twist &signal_);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::mutex buff_mutex_;
};
#endif