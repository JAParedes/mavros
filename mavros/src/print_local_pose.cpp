#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>


const float m_pi = 3.14159265358979323846f;
const float deg2rad = m_pi/180.0f;


ros::Publisher sp_pub;
ros::Subscriber pos_sub;
ros::ServiceClient mode_client;


void callbackPositionRead(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped poseTemp = *msg;
	tf::Quaternion q(
			poseTemp.pose.orientation.y,
			poseTemp.pose.orientation.x,
			-poseTemp.pose.orientation.z,
			poseTemp.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	yaw = yaw/deg2rad;
	ROS_INFO("x_loc: %f m, y_loc: %f m, z_loc: %f m, yaw_loc: %f deg\n", poseTemp.pose.position.y,
poseTemp.pose.position.x, -poseTemp.pose.position.z, yaw);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "print_local_pose");
	
	ros::NodeHandle nh;

	pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav100/mavros/local_position/pose",1, callbackPositionRead);

	ros::spin();

  	return 0;
}

