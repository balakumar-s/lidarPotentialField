//potential_field.cpp
//subscribes to pointcloud data  and implements potential_field
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc,char**argv)
{
	ros::init(argc,argv,"potential_field");
	ros::NodeHandle n;
	ros::Subscriber scan_sub=n.subscribe("/scan",1,callback);

}

void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::LaserScan scan_filtered=*scan_in;
	int total_points=(scan_filtered.angle_max-scan_filtered.angle_min)/scan_filtered.angle_increment;
	for (int i=0;i<total_points;i++)
	{
		if(scan_filtered.ranges[i]==0)
		{		
			scan_filtered.ranges[i]=5;
		}
	}
	for (int i=0;i<total_points;i++)
	{
		
	}
}