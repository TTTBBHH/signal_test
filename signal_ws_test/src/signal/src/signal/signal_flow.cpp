#include"signal/signal_flow.hpp"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "signal/signalpublisher.hpp"
#include "signal/detectedsub.hpp"
#include <rockauto_msgs/DetectedObjectArray.h>
#define PORT 502


    Signal::Signal(ros::NodeHandle &nh) : nh_(nh)
    {
		 signal_ptr_=std::make_shared<SignalPublisher>(nh_,"signalPublisher",10);
		 detectedsub_ptr=std::make_shared<Detectedsubscriber>(nh_,"vision_nodelet_manager/sync/fused_msg",10);
		 InitWithConfig();
    }
   
bool Signal::InitWithConfig(){
	 root_path_ = ros::package::getPath("signal");
	 std::string config_file_path = root_path_ + "/config/signal.yaml";
	 YAML::Node config_node = YAML::LoadFile(config_file_path);
     green = config_node["green"].as<int>();
     yellow = config_node["yellow"].as<int>();
     red = config_node["red1"].as<int>();
	 S = config_node["S"].as<int>();
	return true;
}


   bool Signal::updateWithConfig(){
	        Brt_tg = green;
			Brt_ty=yellow;
            Brt_tr= red;
            TempBrt_tg=green;
			TempBrt_ty=yellow;
            TempBrt_tr=red;
	   return true;
   }


    bool Signal::Run()
    {
	 int sd = socket(PF_INET, SOCK_DGRAM, 0);
	 struct sockaddr_in addr;
 	 addr.sin_family = PF_INET;
 	 addr.sin_port = htons(PORT);
 	 inet_aton("192.168.1.5", &addr.sin_addr);
	
     while (1)
	 { 
		updateWithConfig();
		 ros::Rate rate(1);

			while (Brt_tg > 0 )   //当前被动状态为绿灯
			{
				signallight.angular.x=1;   //表示当前为绿灯的id
				signallight.angular.y=Brt_tg; //绿灯的当前倒计时
				signal_ptr_->Publish(signallight);  //当前绿灯信息发布
				uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 				size_t dataSize1 = sizeof(data1);
 				sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
				LOG(INFO) << "-BRT车道(南北)绿灯 " <<Brt_tg-- ;
  	            LOG(INFO) << "此时信号灯为绿灯 "  ;
				Car_V=detectedsub_ptr->get_V();
				Car_Dis=detectedsub_ptr->get_D();
				Flag=detectedsub_ptr->get_F();
				if (Flag == 1&&(Car_Dis / Car_V) >= Brt_tg){
		    		BRT_ldpd(Flag, Car_V,Car_Dis, Brt_tg); //在每秒--调用主动控制策略
				}  
				rate.sleep();	
			}
			uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
			size_t dataSize2 = sizeof(data2);
			sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));
			
			while (Brt_ty > 0 ) //当前被动状态为黄灯
			{     
				signallight.angular.x=2;  //表示当前被动状态为黄灯的id'
				signallight.angular.y=Brt_ty; //黄灯的当前倒计时
				signal_ptr_->Publish(signallight);
				uint8_t data3[] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A}; //打开通道2
				size_t dataSize3 = sizeof(data3);
				sendto(sd, data3, dataSize3, 0, (struct sockaddr *)&addr, sizeof(addr));
                LOG(INFO) << "-BRT车道(南北)黄灯 " << Brt_ty-- ;
  	            LOG(INFO) << "此时信号灯为黄灯 "  ;
				Car_V=detectedsub_ptr->get_V();
				Car_Dis=detectedsub_ptr->get_D();
				Flag=detectedsub_ptr->get_F();
				if (Flag == 1){
					BRT_hhdpd(Flag, Car_V, Car_Dis, Brt_ty);  //在每秒--调用主动控制策略
				}
				rate.sleep();	
			}
			uint8_t data4[] = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA}; //关闭通道2
			size_t dataSize4 = sizeof(data4);
			sendto(sd, data4, dataSize4, 0, (struct sockaddr *)&addr, sizeof(addr));

			while (Brt_tr > 0)  //当前被动状态为红灯
			{     
				signallight.angular.x=3; //表示当前被动状态为红灯的id'
				signallight.angular.y=Brt_tr;//红灯的当前倒计时
				signal_ptr_->Publish(signallight);\
				uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
				size_t dataSize5 = sizeof(data5);
				sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
				LOG(INFO) << "-BRT车道(南北) 红灯 " <<  Brt_tr-- ;
			    LOG(INFO) << "此时信号灯为红灯 "  ;
				Car_V=detectedsub_ptr->get_V();
				Car_Dis=detectedsub_ptr->get_D();
				Flag=detectedsub_ptr->get_F();
				if (Flag == 1&&(Car_Dis / Car_V) <= Brt_tr){
					BRT_hdpd(Flag, Car_V, Car_Dis, Brt_tr); //在每秒--调用主动控制策略
				}  
				rate.sleep();
			}
			uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
			size_t dataSize6 = sizeof(data6);
			sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
		} 
		}


//绿色信号灯优先控制策略
    //输入是否检测到-Brt_flag；车速-Brt_V；车辆距离-Brt_dis；信号灯时间-Brt_tg；
void Signal::BRT_ldpd (int Brt_flag, float Brt_V, float Brt_dis, int Brt_tg){  
	 ros::Rate rate(1);
	if (Brt_flag == 1)
	{
		LOG(INFO) << "顺利检测到BRT车辆 " ;
		LOG(INFO) << "检测到车速 "<<Brt_V  ;
		LOG(INFO) << "检测到车辆距离 "<<Brt_dis  ;
		T = Brt_dis / Brt_V;               //到达停车线时间T
		if (T > (Brt_tg + TempBrt_ty))                //绿色信号灯时间Brt_tg ,表示无法正常通行，需进行策略调整
		{
			if (T < (TempBrt_tg+TempBrt_ty))                          //33=30+3 黄灯也可以通行 ，表示改变策略可以达到优先通行的要求
			{
				Brt_tg = T-TempBrt_ty;                   //3-黄灯   在实际中应该可以不用-3 相当于留个安全余量
				while (Brt_tg > 0)
				{
				signallight.angular.x=1;   //表示调整当前为绿灯的id
				signallight.angular.y=Brt_tg; //绿灯的当前倒计时
				 signal_ptr_->Publish(signallight);  //当前绿灯信息发布
				LOG(INFO) << "-BRT车道(南北)绿灯 " <<Brt_tg-- ;
				rate.sleep();
				}
			}
			else
			{
			    LOG(INFO) << "不做变化，停车等待 " ;
			}
		}
		else
		{
			 LOG(INFO) << "此时可以顺利通过信号灯 " ;
		}
	}
	else
	{
		LOG(INFO) << "未检测到BRT车辆 " ;
	}
}
//黄色信号灯优先控制策略
     //输入是否检测到-Brt_flag；车速-Brt_V；车辆距离-Brt_dis；信号灯时间-Brt_tg；
void Signal::BRT_hhdpd(int Brt_flag, float Brt_V, float Brt_dis, int Brt_ty)
{
    ros::Rate rate(1);
	if (Brt_flag == 1)
	{
		LOG(INFO) << "顺利检测到BRT车辆 " ;
		LOG(INFO) << "检测到车速 "<<Brt_V  ;
		LOG(INFO) << "检测到车辆距离 "<<Brt_dis ;
		T = Brt_dis / Brt_V;
        if(T>Brt_ty){  //通过所需要时间大于黄灯剩余时间，调用策略
				if(T <=(TempBrt_tg+ TempBrt_ty)){ //可以通过调整策略达到目的
					Brt_tg = T - TempBrt_ty;   
					Brt_ty=TempBrt_ty;
					while (Brt_tg > 0)
					{
					signallight.angular.x=1;   //表示调整当前为绿灯的id
					signallight.angular.y=Brt_tg; //绿灯的当前倒计时
					signal_ptr_->Publish(signallight);  //当前绿灯信息发布
						LOG(INFO) << "-BRT车道(南北)绿灯 " <<Brt_tg-- ;
						rate.sleep();	
					}
						while (Brt_ty > 0)
					{
					   signallight.angular.x=2;   //表示调整当前为黄灯的id
					   signallight.angular.y=Brt_ty; //黄灯的当前倒计时
					   signal_ptr_->Publish(signallight);  //当前黄灯信息发布
					   LOG(INFO) << "-BRT车道(南北)黄灯 " <<Brt_ty-- ;
					   rate.sleep();	
					}
				}else{
					  LOG(INFO) << "不做变化，停车等待 ";
				}
		}else{
        LOG(INFO) << "不做变化，正常通行 ";          
		}
	}else{
		LOG(INFO) << "未检测到BRT车辆 ";
	}
}
//红色信号灯优先控制策略
        //输入是否检测到-Brt_flag；车速-Brt_V；车辆距离-Brt_dis；信号灯时间-Brt_tg；
void Signal::BRT_hdpd(int Brt_flag, float Brt_V, float Brt_dis, int Brt_tr)
{
	int sd = socket(PF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr;
 	addr.sin_family = PF_INET;
 	addr.sin_port = htons(PORT);
 	inet_aton("192.168.1.5", &addr.sin_addr);
	ros::Rate rate(1);
	if (Brt_flag == 1)
	{
        LOG(INFO) << "顺利检测到BRT车辆 " ;
		LOG(INFO) << "检测到车速 "<<Brt_V  ;
		LOG(INFO) << "检测到车辆距离 "<<Brt_dis ;
		T = Brt_dis / Brt_V;
		if (T <=Brt_tr)        //表示无法正常通过，需要改变策略
		{
				if ((TempBrt_tr- Brt_tr) >= 10)    //是否满足非BRT车道的绿灯最小时间：10
					{
                       if(T<=(TempBrt_tg+TempBrt_ty)){
						  uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
						  size_t dataSize6 = sizeof(data6);
						  sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
						  Brt_tg = T - TempBrt_ty;             //减去黄灯时间   理论上这里T一定大于3s即黄灯时间，因为100m检测，按80km/h计算也满足
						  Brt_ty=TempBrt_ty;
                          while (Brt_tg > 0)
						  {
							signallight.angular.x=1;   //表示调整当前为绿灯的id
							signallight.angular.y=Brt_tg; //绿灯的当前倒计时
							signal_ptr_->Publish(signallight);  //当前绿灯信息发布
							uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 							size_t dataSize1 = sizeof(data1);
 							sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
							LOG(INFO) << "-BRT车道(南北)绿灯 " <<Brt_tg-- ;
							rate.sleep();	
						  }
							uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
							size_t dataSize2 = sizeof(data2);
							sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));
                          while (Brt_ty > 0)
						  {
							signallight.angular.x=2;   //表示调整当前为黄灯的id
							signallight.angular.y=Brt_ty; //黄灯的当前倒计时
							signal_ptr_->Publish(signallight);  //当前黄灯信息发布
							uint8_t data3[] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A}; //打开通道2
							size_t dataSize3 = sizeof(data3);
							sendto(sd, data3, dataSize3, 0, (struct sockaddr *)&addr, sizeof(addr));
							LOG(INFO) << "-BRT车道(南北)黄灯 " <<Brt_ty-- ;
							rate.sleep();	
						  }
						  	uint8_t data4[] = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA}; //关闭通道2
							size_t dataSize4 = sizeof(data4);
							sendto(sd, data4, dataSize4, 0, (struct sockaddr *)&addr, sizeof(addr));
					   }else{
						 LOG(INFO) << "不做变化，停车等待" ;  
					   }
					}
					
				else
				{
					if (0 <= (TempBrt_tr- Brt_tr) < 10){
						while(20 < Brt_tr <= 30)
						{
						LOG(INFO) << "-BRT车道(南北)红灯 " <<Brt_tr-- ;
						rate.sleep();
						}
						uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
						size_t dataSize6 = sizeof(data6);
						sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
						T = Brt_dis / Brt_V;
						if(T<=(TempBrt_tg+TempBrt_ty)){
							Brt_tg = T - TempBrt_ty;
						  	Brt_ty=TempBrt_ty;
							while (Brt_tg > 0)
							{
							signallight.angular.x=1;   //表示调整当前为绿灯的id
							signallight.angular.y=Brt_tg; //绿灯的当前倒计时
							signal_ptr_->Publish(signallight);  //当前绿灯信息发布
							uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 							size_t dataSize1 = sizeof(data1);
 							sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
							LOG(INFO) << "-BRT车道(南北)绿灯 " <<Brt_tg-- ;
							rate.sleep();	
							}
							uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
							size_t dataSize2 = sizeof(data2);
							sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));
                        	while (Brt_ty > 0)
							{
							signallight.angular.x=2;   //表示调整当前为黄灯的id
							signallight.angular.y=Brt_ty; //黄灯的当前倒计时
							signal_ptr_->Publish(signallight);  //当前黄灯信息发布
							uint8_t data3[] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A}; //打开通道2
							size_t dataSize3 = sizeof(data3);
							sendto(sd, data3, dataSize3, 0, (struct sockaddr *)&addr, sizeof(addr));
							LOG(INFO) << "-BRT车道(南北)黄灯 " <<Brt_ty-- ;
							rate.sleep();	
							}
							uint8_t data4[] = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA}; //关闭通道2
							size_t dataSize4 = sizeof(data4);
							sendto(sd, data4, dataSize4, 0, (struct sockaddr *)&addr, sizeof(addr));
						}	
					}
					else{
						 LOG(INFO) << "不做变化，停车等待" ;  
					   }
				}
		}else{
		 LOG(INFO) << "此时BRT车辆可以顺利通过路口 " ;
		}
	}else{
     LOG(INFO) << "未检测到BRT车辆 " ;
	}
}
