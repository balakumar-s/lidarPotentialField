//potential_field.cpp
//subscribes to Laserscan data  and implements artificial potential field and publishes resultant force direction and magnitude.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include "laser_geometry/laser_geometry.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"


//Defining constants
#define PI 3.14
#define USER_FORCE  1// Change this to priortize goal
#define ROBOT_RADIUS 0.2// Change this to robot's dimensions.
#define MAX_RANGE 1
//Declaring functions
void callback(const sensor_msgs::LaserScan::ConstPtr&);
void userCallback(const std_msgs::Float32&);
// Declaring global variables
float user_input=180;
ros::Publisher theta_pub,f_pub,pcl_pub;

int main(int argc,char**argv)
{
	ros::init(argc,argv,"potential_field");
	ros::NodeHandle n;
	//Subscribing to user input which is the goal direction
	ros::Subscriber user_sub=n.subscribe("/tunnel/user_input",1,userCallback);
	//Subscribing to LIDAR data.
	ros::Subscriber scan_sub=n.subscribe("/scan",1,callback);
	//Publishing the resultant force magnitude and direction.
	pcl_pub=n.advertise<sensor_msgs::PointCloud2>("/pcl_lidar",1);
	theta_pub=n.advertise<std_msgs::Float32>("/tunnel/direction",1);
	f_pub=n.advertise<std_msgs::Float32>("/tunnel/velocity",1);
	ros::spin();
	return 0;
}

void userCallback(const std_msgs::Float32& user_data)
{
	user_input=user_data.data;
	
}
// The algorithm is run inside the below function so that it is run at the speed of the Lidar scanner.
void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::LaserScan scan_filtered=*scan_in;
	int total_points=(scan_filtered.angle_max-scan_filtered.angle_min)/scan_filtered.angle_increment;
	float force,theta;
	for (int i=0;i<total_points;i++)
	{
		//Remove inf values from LIDAR data
		if(scan_filtered.ranges[i]==0)
		{		
			scan_filtered.ranges[i]=MAX_RANGE;
		}
		if(scan_filtered.ranges[i]>MAX_RANGE)
		{		
			scan_filtered.ranges[i]=MAX_RANGE;
		}
		// Make the robot a point by subtracting the robot radius from the reading.
		if(scan_filtered.ranges[i]>ROBOT_RADIUS)
		{
		scan_filtered.ranges[i]=scan_filtered.ranges[i]-ROBOT_RADIUS;
		}
		// If there are 0 readings, make them the LIDAR sensor's maximum range.
	}
	//Transforming to pointcloud
	sensor_msgs::PointCloud2 filtered_cloud;
	pcl::PointCloud<pcl::PointXYZ> input_cloud;

	laser_geometry::LaserProjection projector;
	projector.projectLaser(scan_filtered,filtered_cloud);
	pcl::fromROSMsg(filtered_cloud,input_cloud);
	pcl_pub.publish(filtered_cloud);
	// Variables for potential field are initialized.
	float force_x=0;
	float force_y=0;
	float _force_x=0;
	float _force_y=0;
	float user_force=USER_FORCE;
	// The goal attraction is obtained from the user input subscriber
	float user_x=user_force*cos(user_input*PI/180);
	float user_y=user_force*sin(user_input*PI/180);
	int non_zero_points=1;
	//Forces are computed for the LIDAR data.
	if(input_cloud.is_dense==false)
	{
		//ROS_INFO("true");
	for (int i=0;i<total_points;i++)
	{ 
		if(scan_filtered.ranges[i]!=MAX_RANGE)
		{
		float x_lidar = input_cloud[i].x;
		float y_lidar = input_cloud[i].y;
	    //ROS_INFO("x:%0.2f y:%0.2f",x_lidar,y_lidar);

	  if(scan_filtered.ranges[i]>0)//&&(x_lidar==x_lidar)&&(y_lidar==y_lidar))
	  {
		  
		  //Transforming to robot frame
		  float x=x_lidar*cos(-90*PI/180)-y_lidar*sin(-90*PI/180);
		  float y=x_lidar*sin(-90*PI/180)+y_lidar*cos(-90*PI/180);
		  //ROS_INFO("x:%0.2f y:%0.2f",x,y);
		  // The scale is inverted to make nearer obstacles have higher force and the scale is reduced to 1.
		  if(fabs(x)>0)
		  	{
		  		x=1/x;
    	  	}
		  if(fabs(y)>0)
		  {
		  		y=1/y;
		  }
		  
		  

		  force_x+=x;
		  force_y+=y;
		  _force_x+=x;
		  _force_y+=y;
		  non_zero_points++;
	  }
	}
	}
	}
   
    float norm_factor=sqrt(pow(_force_x,2)+pow(_force_y,2));

	//ROS_INFO("x:%0.2f y:%0.2f",_force_x,_force_y);
	total_points=non_zero_points;
	// The resultant direction is calculated
	float resultant_x=0;
	float resultant_y=0;
	if(norm_factor!=0)
	{
		resultant_x=(_force_x/norm_factor)+1;
		resultant_y=(_force_y/norm_factor)+0.02;
	}
	//ROS_INFO("x:%0.2f y:%0.2f",resultant_x,resultant_y);

    theta=atan2(-(resultant_y+user_y),-(resultant_x+user_x))*180/PI;
    // The range is shifted to 0-359 degrees.
    if(theta<0)
    {
    	theta=360+theta;
    }
   /* theta=theta+90;
    if(theta>359)
    {
    	theta=theta-360;
    }
    */

    //force vectors are normalized
    //force_x=(force_x/total_points)-user_x;//+user_x;
	//force_y=(force_y/total_points)-user_y;//+user_y;
	force_x=resultant_x+user_x;
	force_y=resultant_y+user_y;
	//Resultant force vector magnitude is computed
	force=sqrt(pow(force_x,2)+pow(force_y,2));
	//Force magnitude and direction is published.
    std_msgs::Float32 force_,theta_;
	force_.data=100*(force)/sqrt(2*MAX_RANGE);
	if(force_.data>100)
	{
		force_.data=100;
	}
	theta_.data=theta;
	f_pub.publish(force_);
	theta_pub.publish(theta_);
}
