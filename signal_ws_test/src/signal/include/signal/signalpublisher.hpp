#ifndef SIGNAL_PUBLISHER_GNSS_PUBLISHER_HPP_
#define SIGNAL_PUBLISHER_GNSS_PUBLISHER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class SignalPublisher {
 public:
  SignalPublisher(ros::NodeHandle& nh, std::string topic_name,
                size_t buff_size);
  SignalPublisher() = default;

  void Publish(geometry_msgs::Twist& signal_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

#endif