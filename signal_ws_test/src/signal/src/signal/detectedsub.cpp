#include "signal/detectedsub.hpp"
#include "signal/signal_flow.hpp"
#include <rockauto_msgs/DetectedObjectArray.h>

Detectedsubscriber::Detectedsubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size) : nh_(nh)
{
  subscriber_2 = nh_.subscribe(topic_name, buff_size, &Detectedsubscriber::detected_callback, this);
}

void Detectedsubscriber::detected_callback(
    const rockauto_msgs::DetectedObjectArray::ConstPtr &detected)
{
  buff_mutex_.lock();
  flag_ = 1;
  // Car_V_ = detected->objects.velocity;
  // Car_Dis_= std::sqrt(detected->objects.position.x * detected->objects.position.x+ detected->objects.position.y* detected->objects.position.y);
  for (auto obj = detected->objects.begin(); obj != detected->objects.end(); ++obj)
  {
    Car_V_ = obj->velocity.linear.x;
    Car_Dis_ = obj->pose.position.x;

  }
  buff_mutex_.unlock();
}
