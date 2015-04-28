//potential_field.cpp
//subscribes to pointcloud data  and implements potential_field
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <std_msgs/Float32.h>
#define PI 3.14
void callback(const sensor_msgs::LaserScan::ConstPtr&);
ros::Publisher theta_pub,f_pub;
int main(int argc,char**argv)
{
	ros::init(argc,argv,"potential_field");
	ros::NodeHandle n;
	ros::Subscriber scan_sub=n.subscribe("/scan",1,callback);
	theta_pub=n.advertise<std_msgs::Float32>("/tunnel/direction",1);
	f_pub=n.advertise<std_msgs::Float32>("/tunnel/velocity",1);
	ros::spin();
	return 0;
}

void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::LaserScan scan_filtered=*scan_in;
	int total_points=(scan_filtered.angle_max-scan_filtered.angle_min)/scan_filtered.angle_increment;
	/*float forces[360][2],force_v[2],force,theta;
	
	int parm = 1233;
	force_v[0] =0;
	force_v[1] =0;
	*/
	float force,theta;
	for (int i=0;i<total_points;i++)
	{
		if(scan_filtered.ranges[i]==0)
		{		
			scan_filtered.ranges[i]=5;
		}
	}
	float force_x=0;
	float force_y=0;
	float _force_x=0;
	float _force_y=0;
	for (int i=0;i<total_points;i++)
	{ 
	  
	  float x = (scan_filtered.ranges[i]*cos(i*PI/180));
	  float y = (scan_filtered.ranges[i]*sin(i*PI/180));
	  x=(5-x)/5;
	  y=(5-y)/5;
	  force_x+=x;
	  force_y+=y;

	  float x = -(scan_filtered.ranges[i]*cos(i*PI/180));
	  float y = -(scan_filtered.ranges[i]*sin(i*PI/180));
	  _force_x+=x;
	  _force_y+=y;

	/*   int x,y;
	   int normal_x,normal_y;
	   x = scan_filtered.ranges[i]*cos(i*PI/180);
	   y = scan_filtered.ranges[i]*sin(i*PI/180);
	   normal_x = x/sqrt(pow(x,2)+pow(y,2));
	   normal_y = y/sqrt(pow(x,2)+pow(y,2));
	   forces[i][0] = ((parm/(pow(0.01*scan_filtered.ranges[i],2)))*normal_x);
	   forces[i][1] = ((parm/(pow(0.01*scan_filtered.ranges[i],2)))*normal_y);
	   force_v[0]+=  forces[i][0];
	   force_v[1]+=  forces[i][1];
	*/ 
	}
	//  force = sqrt(pow(force_v[0],2) + pow(force_v[1],2));
	//  theta = atan2(force_v[1],force_v[0])*PI/180;
    theta=atan2(_force_y,_force_x)*180/PI;
    theta=theta+180;
    force_x=(force_x/total_points);
	force_y=(force_y/total_points);
	//ROS_INFO("force_x:%f force_y: %f",force_x,force_y);
	force=sqrt(pow(force_x,2)+pow(force_y,2));
	ROS_INFO("theta: %f force: %f",theta,force);
	std_msgs::Float32 force_,theta_;
	force_.data=100*(force)/sqrt(2);
	theta_.data=theta;
	f_pub.publish(force_);
	theta_pub.publish(theta_);
}
