#ifndef  SIGNAL_FLOW_HPP_
#define SIGNAL_FLOW_HPP_
#include <ros/ros.h>
#include <ros/package.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include"signal/signalpublisher.hpp"
#include"signal/detectedsub.hpp"
#include<geometry_msgs/Twist.h>
#include <rockauto_msgs/DetectedObjectArray.h>

    class Signal
    {
        public:
        //前端检测参数：
        int Flag;  //1-表示检测到BRT车辆，0-表示未检测到BRT车辆
        float Car_V;//车辆速度
        float Car_Dis;//车辆距离红绿灯距离
      geometry_msgs::Twist signallight;
       public:
        //直行信号灯配时参数
        int green ;
        int yellow;
        int red ;
        int S;
         int T;
         int Brt_tg;      //BRT绿灯
         int Brt_ty;      //BRT黄灯
         int Brt_tr;      //BRT红灯
         
        int TempBrt_tg;      //缓存BRT绿灯
         int TempBrt_ty;      //缓存BRT黄灯
         int TempBrt_tr;      //缓存BRT红灯


         std::string root_path_;
  
        public:
        Signal(ros::NodeHandle &nh);
        bool Run();
        bool InitWithConfig();
        bool updateWithConfig();
        void  BRT_ldpd (int Brt_flag, float Brt_V, float Brt_dis, int Brt_tg);
        void  BRT_hhdpd(int Brt_flag, float Brt_V, float Brt_dis, int Brt_ty);
        void  BRT_hdpd(int Brt_flag, float Brt_V, float Brt_dis, int Brt_tr);
        private:
       std::shared_ptr<SignalPublisher>signal_ptr_;
       std::shared_ptr<Detectedsubscriber>detectedsub_ptr;
        private:
        ros::NodeHandle nh_;
    };
    

#endif