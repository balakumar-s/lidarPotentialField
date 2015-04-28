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
	float forces[360][2],force[2];
	int parm = 1233;
	force = [0,0];
	for (int i=0;i<total_points;i++)
	{
		if(scan_filtered.ranges[i]==0)
		{		
			scan_filtered.ranges[i]=5;
		}
	}
	for (int i=0;i<total_points;i++)
	{ int x,y;
	   
	   x = scan_filtered.ranges[i]*cos(i);
	   y = scan_filtered.ranges[i]*sin(i);
	   normal_x = x/sqrt(pow(x,2)+pow(y,2));
	   normal_y = y/sqrt(pow(x,2)+pow(y,2));
	   forces[i][0] = ((parm/(pow(0.01*scan_filtered.ranges[i],2)))*normal_x);
	   forces[i][1] = ((parm/(pow(0.01*scan_filtered.ranges[i],2)))*normal_y);
	   force[0]+ =  forces[i][0];
	   force[1]+ =  forces[i][1];
	   
	}
	 
}
