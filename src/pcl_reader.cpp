#include<ros/ros.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_full;
ros::Publisher pub_middle;
ros::Publisher pub_left;
ros::Publisher pub_right;

void callback(const PointCloud::ConstPtr& msg){
	BOOST_FOREACH(const pcl::PointXYZ &pt, msg->points)
		printf("%f %f %f\n", pt.x, pt.y, pt.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "pcl_reader");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<PointCloud>("points", 1, callback);
	ros::spin();

	/* never get here */
	return 0;
}
