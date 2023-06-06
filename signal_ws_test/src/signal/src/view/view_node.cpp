#include"view/view_flow.hpp"

int main(int argc,char*argv[])
{
ros::init(argc,argv,"view_node");
ros::NodeHandle nh;
std::shared_ptr<View>view_ptr=std::make_shared<View>(nh);
ros::Rate rate(10);
while(ros::ok()){
     ros::spinOnce();
    view_ptr->Run();
    rate.sleep();
}
return 0;
}