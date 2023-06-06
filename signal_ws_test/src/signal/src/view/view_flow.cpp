#include"view/view_flow.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>

View::View(ros::NodeHandle &nh):nh_(nh) {
                InitWithView();
            viewSub_ptr=std::make_shared<Viewsubscriber>(nh_,"signalPublisher",10);
            }

void View:: InitWithView()
{ 
     CustomWin=cv::Mat(1000,1000,CV_8UC3,cv::Scalar(255,255,255));    
    cv::rectangle(CustomWin,cv::Point(200,100),cv::Point(800,900),cv::Scalar(88,87,86),2); //画矩形(信号灯-BRT直)
    cv::rectangle(CustomWin,cv::Point(680,452),cv::Point(780,546),cv::Scalar(88,87,86),2); //倒计时框
    //BRT车道信号灯
    cv::circle(CustomWin,cv::Point(500, 250),120,cv::Scalar(0,0,0),8);      //画圆
    cv::circle(CustomWin,cv::Point(500, 500),120,cv::Scalar(0,0,0),8);      //画圆
    cv::circle(CustomWin,cv::Point(500, 750),120,cv::Scalar(0,0,0),8);      //画圆
    cv::waitKey(1);
}

void View::Run(){
     UpdateWithView();
}

void View:: UpdateWithView(){
         
        cv::Mat LightWin;
        CustomWin.copyTo(LightWin);
    if( viewSub_ptr->signal_test.angular.x==1){
         int green= viewSub_ptr->signal_test.angular.y;
     cv::circle( LightWin,cv::Point(500, 250),116,cv::Scalar(0,255,0),-1);      //BRT直行绿灯
     cv::putText (LightWin,std::to_string(green),cv::Point(690,519),cv::FONT_HERSHEY_TRIPLEX,2,cv::Scalar(0,255,0));  //BRT直行绿灯
    }
    else if(viewSub_ptr->signal_test.angular.x==2){
        int yellow=viewSub_ptr->signal_test.angular.y;
         cv::circle(LightWin,cv::Point(500, 500),116,cv::Scalar(0,255,255),-1);      //BRT直行黄灯
         cv::putText(LightWin,std::to_string(yellow),cv::Point(690,519),cv::FONT_HERSHEY_TRIPLEX,2,cv::Scalar(255,0,255));  
    }
    else if(viewSub_ptr->signal_test.angular.x==3){
        int red=viewSub_ptr->signal_test.angular.y;
        cv::circle(LightWin,cv::Point(500, 750),116,cv::Scalar(0,0,255),-1);      //BRT直行红灯
        cv::putText(LightWin,std::to_string(red),cv::Point(690,519),cv::FONT_HERSHEY_TRIPLEX,2,cv::Scalar(0,0,255));  
    }
    cv::namedWindow("LightWin", CV_WINDOW_NORMAL); //更改窗口大小 或在创建一个窗口也用这个函数
    cv::imshow("LightWin",LightWin);
    cv::waitKey(1);
}
