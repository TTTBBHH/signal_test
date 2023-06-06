#include<ros/ros.h>
#include"signal/signal_flow.hpp"
#include <rockauto_msgs/DetectedObjectArray.h>

int main(int argc, char *argv[])
{
    std::string root_path = ros::package::getPath("signal");
     google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = root_path + "/log";
    FLAGS_alsologtostderr = 1;
    
    ros::init(argc, argv, "signal_node");
    ros::NodeHandle nh;
    std::shared_ptr<Signal>signal_ptr_=std::make_shared<Signal>(nh);
    ros::Rate rate(10);
    setlocale(LC_ALL,"");
    while (ros::ok())
    {
        ros::spinOnce();
       signal_ptr_->Run();
        rate.sleep();
    }
    return 0;
}
