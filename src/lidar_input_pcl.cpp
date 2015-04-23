//lidar_input_pcl.cpp
//Subcribes to lidar data and publishes pointcloud data.
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

//global variables
laser_geometry::LaserProjection projector_;
ros::Publisher pcl_pub;
sensor_msgs::PointCloud2 cloud,pass_cloud;

void callback (const sensor_msgs::LaserScan::ConstPtr&);

int main (int argc,char**argv)
{
	ros::init(argc,argv,"lidar_input_pcl");
	ros::NodeHandle n;
	pcl_pub=n.advertise<sensor_msgs::PointCloud2>("input/pcl_lidar",1);
	ros::Subscriber laser_sub=n.subscribe("/scan",1,callback);
	ros::spin();
	return(0);
}

void callback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
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
	projector_.projectLaser(scan_filtered,cloud);
	pcl_pub.publish(cloud);

}
