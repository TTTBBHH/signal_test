#include "view/viewsubscriber.hpp"

Viewsubscriber::Viewsubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size): nh_(nh) {
  subscriber_ =nh_.subscribe(topic_name, buff_size, &Viewsubscriber::msg_callback, this);
}

void Viewsubscriber::msg_callback(
    const geometry_msgs::Twist &signal_) {
  buff_mutex_.lock();
   signal_test = signal_; 
  buff_mutex_.unlock();
}