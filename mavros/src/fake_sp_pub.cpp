#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sstream>

int main(int argc, char **argv)
{
	const float deg2rad = M_PI/180.;

  ros::init(argc, argv, "fake_sp_pub");

  ros::Time::init;
	float x, y, z, roll, pitch, yaw;

	ros::NodeHandle nh;
	ros::Publisher sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav100/mavros/setpoint_position/local", 10);
	//ros::Rate loop_rate(10);	
	ros::Rate loop_rate(20);


  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::PoseStamped pose;
	
    //std::stringstream ss;
    //ss << "MSG NUM: " << count;

    ROS_INFO("MSG NUM (SP): %i", count);

		pose.header.seq = count;
		pose.header.stamp = ros::Time::now();
		//pose.header.frame_id = 1;

		// True position: x = 4, y = 6, z = -3
		pose.pose.position.x = 6;
		pose.pose.position.y = 4;
		pose.pose.position.z = 4;

		//True yaw: 45 deg
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = -0.3826834;
		pose.pose.orientation.w = 0.9238795;

    sp_pub.publish(pose);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
