#include<ros/ros.h>
#include <locale>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <thread>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <autoware_msgs/DetectedObjectArray.h>
#define PORT 502

    class demo
    {
      public:
        int green=30;
        int yellow=3;
        int red=30;
        int S;
        int T;
        int Brt_tg;     
        int Brt_ty;    
        int Brt_tr;      
        int TempBrt_tg;     
        int TempBrt_ty;   
        int TempBrt_tr;      
        std::string root_path_;
		float Car_V;
  		float Car_Dis;
  		int Flag;

    /* bool InitWithConfig(){
			root_path_ = ros::package::getPath("signal");
			std::string config_file_path = root_path_ + "/config/signal.yaml";
			YAML::Node config_node = YAML::LoadFile(config_file_path);
  			green = config_node["green"].as<int>();
  			yellow = config_node["yellow"].as<int>();
  			red = config_node["red1"].as<int>();
  			S = config_node["S"].as<int>();
			return true;
		}
		  */
    bool updateWithConfig(){
			Brt_tg = green;
			Brt_ty= yellow;
  			Brt_tr= red;
  			TempBrt_tg=green;
			TempBrt_ty=yellow;
  			TempBrt_tr=red;
			return true;
		}
//绿色信号灯优先控制策略
    void  BRT_ldpd (int Brt_flag, float Brt_V, float Brt_dis, int Brt_tg){
		int sd = socket(PF_INET, SOCK_DGRAM, 0);
		struct sockaddr_in addr;
 		addr.sin_family = PF_INET;
 		addr.sin_port = htons(PORT);
 		inet_aton("192.168.1.5", &addr.sin_addr);
		ros::Rate rate(1);
		if (Brt_flag == 1)
		{
        	ROS_INFO("顺利检测到BRT车辆 ") ;
			ROS_INFO("检测到车速: %f",Brt_V );
			ROS_INFO("检测到车辆距离: %f",Brt_dis);
			T = std::ceil(Brt_dis/Brt_V);
			if(T > (Brt_tg + TempBrt_ty))
			{
				if(T < (TempBrt_tg + TempBrt_ty))
				{
					Brt_tg = T - TempBrt_ty;             //减去黄灯时间   理论上这里T一定大于3s即黄灯时间
        			while (Brt_tg > 0)
					{
						uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 						size_t dataSize1 = sizeof(data1);
 						sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
						ROS_INFO("-BRT车道(南北)绿灯: %d",Brt_tg--);
						rate.sleep();
					}
					uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
					size_t dataSize2 = sizeof(data2);
					sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));
				}
				else
				{
					ROS_INFO("不做变化，停车等待 ");
					while (Brt_tg > 0)
					{
						uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 						size_t dataSize1 = sizeof(data1);
 						sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
						ROS_INFO("-BRT车道(南北)绿灯: %d",Brt_tg--);
						rate.sleep();
					}
					uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
					size_t dataSize2 = sizeof(data2);
					sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));
				}
			}
			else
			{
				ROS_INFO("此时可以顺利通过信号灯 ");
				while (Brt_tg > 0)
				{
					uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 					size_t dataSize1 = sizeof(data1);
 					sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
					ROS_INFO("-BRT车道(南北)绿灯: %d",Brt_tg--);
					rate.sleep();
				}
				uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
				size_t dataSize2 = sizeof(data2);
				sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));
			}	
		}
	}

    void  BRT_hhdpd(int Brt_flag, float Brt_V, float Brt_dis, int Brt_ty);

//红色信号灯优先控制策略
    void BRT_hdpd(int Brt_flag, float Brt_V, float Brt_dis, int Brt_tr){
		int sd = socket(PF_INET, SOCK_DGRAM, 0);
		struct sockaddr_in addr;
 		addr.sin_family = PF_INET;
 		addr.sin_port = htons(PORT);
 		inet_aton("192.168.1.5", &addr.sin_addr);
		ros::Rate rate(1);
		if (Brt_flag == 1)
		{
        	ROS_INFO("顺利检测到BRT车辆 ") ;
			ROS_INFO("检测到车速: %f",Brt_V );
			ROS_INFO("检测到车辆距离: %f",Brt_dis);
			T = std::ceil(Brt_dis/Brt_V);
			if (T <=Brt_tr)
			{
				if ((TempBrt_tr- Brt_tr) >= 10)
				{
					if(T<=(TempBrt_tg+TempBrt_ty))
					{
						uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
						size_t dataSize6 = sizeof(data6);
						sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
						Brt_tg = T - TempBrt_ty;             //减去黄灯时间   理论上这里T一定大于3s即黄灯时间
						Brt_ty=TempBrt_ty;
        				while (Brt_tg > 0)
						{
							uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 							size_t dataSize1 = sizeof(data1);
 							sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)绿灯: %d",Brt_tg--);
							rate.sleep();
						}
						uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
						size_t dataSize2 = sizeof(data2);
						sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));

						while (Brt_ty > 0)
						{		
							uint8_t data3[] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A}; //打开通道2
							size_t dataSize3 = sizeof(data3);
							sendto(sd, data3, dataSize3, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)黄灯: %d",Brt_ty--);
							rate.sleep();	
						}
						uint8_t data4[] = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA}; //关闭通道2
						size_t dataSize4 = sizeof(data4);
						sendto(sd, data4, dataSize4, 0, (struct sockaddr *)&addr, sizeof(addr));

						while(Brt_tr > 0)
							{
							uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
							size_t dataSize5 = sizeof(data5);
							sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)红灯: %d",Brt_tr--);
							rate.sleep();
							}
							//关闭通道3
							sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
					}
					else
					{
						ROS_INFO("不做变化，停车等待"); 
						while(Brt_tr > 0)
							{
							uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
							size_t dataSize5 = sizeof(data5);
							sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)红灯: %d",Brt_tr--);
							rate.sleep();
							}
							uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
							size_t dataSize6 = sizeof(data6);
							sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
					}
				}
				else
				{
					if ((TempBrt_tr- Brt_tr) >= 0 && TempBrt_tr- Brt_tr < 10)
					{
						if(T<=(TempBrt_tg+TempBrt_ty))
						{
							while(Brt_tr > 20 && Brt_tr <= 30)
							{
							uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
							size_t dataSize5 = sizeof(data5);
							sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)红灯: %d",Brt_tr--);
							rate.sleep();
							}
							uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
							size_t dataSize6 = sizeof(data6);
							sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
							ros::spinOnce();
							T = std::ceil(Brt_dis/Brt_V);
							Brt_tg = T - TempBrt_ty;
							Brt_ty=TempBrt_ty;
							while (Brt_tg > 0)
							{
								uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 								size_t dataSize1 = sizeof(data1);
 								sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
								ROS_INFO("-BRT车道(南北)绿灯: %d",Brt_tg--);
								rate.sleep();	
							}
							uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
							size_t dataSize2 = sizeof(data2);
							sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));
							while (Brt_ty > 0)
							{
								uint8_t data3[] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A}; //打开通道2
								size_t dataSize3 = sizeof(data3);
								sendto(sd, data3, dataSize3, 0, (struct sockaddr *)&addr, sizeof(addr));
								ROS_INFO("-BRT车道(南北)黄灯: %d",Brt_ty--);
								rate.sleep();	
							}
							uint8_t data4[] = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA}; //关闭通道2
							size_t dataSize4 = sizeof(data4);
							sendto(sd, data4, dataSize4, 0, (struct sockaddr *)&addr, sizeof(addr));
							while(Brt_tr > 0)
							{
							uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
							size_t dataSize5 = sizeof(data5);
							sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)红灯: %d",Brt_tr--);
							rate.sleep();
							}
							 //关闭通道3
							sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
						}
						else
						{
							ROS_INFO("不做变化，停车等待"); 
							while(Brt_tr > 0)
							{
							uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
							size_t dataSize5 = sizeof(data5);
							sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)红灯: %d",Brt_tr--);
							rate.sleep();
							}
							uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
							size_t dataSize6 = sizeof(data6);
							sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
						}
					}
				}
			}
			else
			{
				ROS_INFO("此时BRT车辆可以顺利通过路口 ");
				while(Brt_tr > 0)
							{
							uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
							size_t dataSize5 = sizeof(data5);
							sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
							ROS_INFO("-BRT车道(南北)红灯: %d",Brt_tr--);
							rate.sleep();
							}
							uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
							size_t dataSize6 = sizeof(data6);
							sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
			}
		}
	}
		

		void signalCallback(const autoware_msgs::DetectedObjectArrayConstPtr &msg)
		{   
    
    		for (auto obj = msg->objects.begin(); obj != msg->objects.end(); ++obj)
  			{
				if(obj->pose.position.x > 0 && obj->pose.position.x < 3){
				Flag=1;
    			Car_V= 2;
    			Car_Dis = std::abs(obj->pose.position.y);
    			ROS_INFO("Car_Dis: %f ",Car_Dis) ;
				ROS_INFO("Car_V: %f ",Car_V) ;
				ROS_INFO("Flag: %d",Flag) ;
				}
  			}
		}
  

	bool run()
	{	
  	int sd = socket(PF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr;
 	addr.sin_family = PF_INET;
 	addr.sin_port = htons(PORT);
 	inet_aton("192.168.1.5", &addr.sin_addr);
  
  	while (ros::ok())
	{
		ros::spinOnce();
		updateWithConfig();
		ros::Rate rate(1);
		while(Brt_tg > 0)//当前被动状态为绿灯
		{
			ros::spinOnce();
			uint8_t data1[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}; //打开通道1
 			size_t dataSize1 = sizeof(data1);
 			sendto(sd, data1, dataSize1, 0, (struct sockaddr *)&addr, sizeof(addr));
			ROS_INFO("-BRT车道(南北)绿灯: %d",Brt_tg--) ;
  	        ROS_INFO("此时信号灯为绿灯 ");
			if (Flag == 1){
		    	BRT_ldpd(Flag, Car_V,Car_Dis, Brt_tg); 
				Brt_tg = 0;
				}  
			rate.sleep();	
		}
			uint8_t data2[] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}; //关闭通道1
			size_t dataSize2 = sizeof(data2);
			sendto(sd, data2, dataSize2, 0, (struct sockaddr *)&addr, sizeof(addr));

		while(Brt_ty > 0)//当前被动状态为黄灯
		{
			uint8_t data3[] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A}; //打开通道2
			size_t dataSize3 = sizeof(data3);
			sendto(sd, data3, dataSize3, 0, (struct sockaddr *)&addr, sizeof(addr));
            ROS_INFO("-BRT车道(南北)黄灯: %d",Brt_ty--) ;
  	        ROS_INFO("此时信号灯为黄灯 ");
			rate.sleep();
		}
			uint8_t data4[] = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA}; //关闭通道2
			size_t dataSize4 = sizeof(data4);
			sendto(sd, data4, dataSize4, 0, (struct sockaddr *)&addr, sizeof(addr));
		
		
		while (Brt_tr > 0)  //当前被动状态为红灯
		{     
			ros::spinOnce();
			uint8_t data5[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A}; //打开通道3
			size_t dataSize5 = sizeof(data5);
			sendto(sd, data5, dataSize5, 0, (struct sockaddr *)&addr, sizeof(addr));
			ROS_INFO("-BRT车道(南北)红灯: %d",Brt_tr--) ;
			ROS_INFO("此时信号灯为红灯 ");
			if(Flag == 1){
				BRT_hdpd(Flag, Car_V, Car_Dis, Brt_tr);
				Brt_tr = 0;
			}
			rate.sleep();
		}
			uint8_t data6[] = {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA}; //关闭通道3
			size_t dataSize6 = sizeof(data6);
			sendto(sd, data6, dataSize6, 0, (struct sockaddr *)&addr, sizeof(addr));
	} 
}	
};


int main(int argc, char **argv){
	std::locale::global(std::locale(""));
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;
	demo obj;
	
    ros::Subscriber sub = n.subscribe("fuse_msg_topic",1000,&demo::signalCallback, &obj);
    obj.run();
   
    return 0;
}
