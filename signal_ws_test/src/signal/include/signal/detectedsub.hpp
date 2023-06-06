#ifndef  DETECTED_SUB_HPP_
#define DETECTED_SUB_HPP_
#include <ros/ros.h>
#include <rockauto_msgs/DetectedObjectArray.h>
#include <mutex>

class Detectedsubscriber {
public:
  Detectedsubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  Detectedsubscriber() = default;

public:
  
    float get_V(){
    return Car_V_;
  }    
  float get_D(){
    return Car_Dis_;
  }
float get_F(){
    return flag_;
  }

private:
  void detected_callback(const rockauto_msgs::DetectedObjectArray::ConstPtr&detected);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_2;
  std::mutex buff_mutex_;
float Car_V_=0;
float Car_Dis_=0;
float flag_=0;
};
#endif