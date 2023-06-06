#ifndef VIEW_FLOW_HPP_
#define VIEW_FLOW_HPP_

#include<ros/ros.h>
#include <opencv2/opencv.hpp>
   //添加订阅头文件
#include"view/viewsubscriber.hpp"

class View
{
   public:                                         
           View(ros::NodeHandle &nh);
            void Run();
           void  InitWithView();
            void  UpdateWithView();
  public:
            cv::Mat CustomWin;
            std::shared_ptr<Viewsubscriber>viewSub_ptr;
     private : 
            ros::NodeHandle nh_;
};         

#endif