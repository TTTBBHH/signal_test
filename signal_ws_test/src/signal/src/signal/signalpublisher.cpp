#include "signal/signalpublisher.hpp"

SignalPublisher::SignalPublisher(ros::NodeHandle& nh, std::string topic_name,
                              size_t buff_size)
    : nh_(nh) {
  publisher_ = nh_.advertise<geometry_msgs::Twist>(topic_name, buff_size);
}

void SignalPublisher:: Publish(geometry_msgs::Twist& signal_ptr) {
  publisher_.publish( signal_ptr);
}