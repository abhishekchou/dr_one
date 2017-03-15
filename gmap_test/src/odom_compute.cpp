#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

using namespace ros;



//void odomCallback(const sensor_msgs::Imu::ConstPtr& msg)
//{
    
//}

int main(int argc, char* argv[])
{
    init(argc,argv,"odom_compute");
    NodeHandle n;
    NodeHandle nPrivate("~");	//private node handle to get the relative parameters of the node    

//    Subscriber fcu_pose = n.subscribe<sensor_msgs::Imu >("/mavros/imu/data",10,odomCallback);
//    Publisher base_odom = n.advertise<sensor_msgs::Imu >("/dr_one/odom",1);

    Rate loop_rate(20);

	while(ok())
	{
		spinOnce();        
        loop_rate.sleep();
	}
	return 0;
}
