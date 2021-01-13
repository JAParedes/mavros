#include <stdio.h>
#include <thread>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <mutex>

std::mutex mtx;

static FILE *wp_file = NULL;

float sp_x = 0.0f;
float sp_y = 0.0f;
float sp_z = 0.0f;

float sp_qx = 0.0f;
float sp_qy = 0.0f;
float sp_qz = 0.0f;
float sp_qw = 1.0f;

float wp_radius = 0.25f;
float wp_wait_time = 0.0f;

const float m_pi = 3.14159265358979323846f;
const float deg2rad = m_pi/180.0f;


ros::Publisher sp_pub;
ros::Subscriber pos_sub;
//ros::ServiceClient arming_client;
ros::ServiceClient mode_client;

void callbackPositionRead(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped poseTemp = *msg;
	static float spp_x = 0.0f;
	static float spp_y = 0.0f;
	static float spp_z = 0.0f;
	static float spp_yaw = 0.0f;
	static int count = 0;
	if (count == 0)
	{
		fscanf(wp_file, "%f,%f,%f,%f", &spp_x, &spp_y, &spp_z, &spp_yaw);
		tf::Quaternion spp_q_init = tf::createQuaternionFromYaw(deg2rad*spp_yaw);
		mtx.lock();
		sp_x = spp_x;
		sp_y = spp_y;
		sp_z = spp_z;
		sp_qx = spp_q_init.x();
		sp_qy = spp_q_init.y();
		sp_qz = spp_q_init.z();
		sp_qw = spp_q_init.w();
		mtx.unlock();
		count++;
	}
	float err_x_sq = (poseTemp.pose.position.x - spp_x)*(poseTemp.pose.position.x - spp_x);
	float err_y_sq = (poseTemp.pose.position.y - spp_y)*(poseTemp.pose.position.y - spp_y);
	float err_z_sq = (poseTemp.pose.position.z - spp_z)*(poseTemp.pose.position.z - spp_z);
	if ((err_x_sq + err_y_sq + err_z_sq)<wp_radius)
	{
		if(fscanf(wp_file, "%f,%f,%f,%f", &spp_x, &spp_y, &spp_z, &spp_yaw) == EOF){
			//mavros_msgs::CommandBool arm_cmd;
    			//arm_cmd.request.value = false;
			//arming_client.call(arm_cmd);
			mavros_msgs::SetMode land_set_mode;
    			land_set_mode.request.custom_mode = "AUTO.LAND";
			if( mode_client.call(land_set_mode) &&
				land_set_mode.response.mode_sent){
				ROS_INFO("Landing... \n");
			}
		}else{
			ROS_INFO("Arrived at waypoint! \n");
			if (wp_wait_time>=0.01f){
				ros::Duration(wp_wait_time).sleep();
			}
			ROS_INFO("Next waypoint... \n");
			tf::Quaternion spp_q = tf::createQuaternionFromYaw(deg2rad*spp_yaw);
			mtx.lock();
			sp_x = spp_x;
			sp_y = spp_y;
			sp_z = spp_z;
			sp_qx = spp_q.x();
			sp_qy = spp_q.y();
			sp_qz = spp_q.z();
			sp_qw = spp_q.w();
			mtx.unlock();
		}
	}
}

void callbackPostSetpoints(const ros::TimerEvent& event)
{
	static geometry_msgs::PoseStamped pose;
    static int count = 0;
	if (count == 0)
	{
		pose.pose.position.x = 0.0f;
		pose.pose.position.y = 0.0f;
		pose.pose.position.z = 0.0f;
		count++;
	}
	mtx.lock();
	pose.pose.position.x = sp_x;
	pose.pose.position.y = sp_y;
	pose.pose.position.z = sp_z;
	pose.pose.orientation.x = sp_qx;
	pose.pose.orientation.y = sp_qy;
	pose.pose.orientation.z = sp_qz;
	pose.pose.orientation.w = sp_qw;
	mtx.unlock();
	sp_pub.publish(pose);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setpoint_publisher");
	ros::Time::init();

	ros::Duration(5).sleep();

	wp_file = fopen("/home/umich-aero-2020/catkin_mavros_ws/waypoints.txt","r");

	ROS_INFO("Setpoint publisher online!\n");

	ros::NodeHandle nh;
	//nh.param("~wp_rad", wp_radius, 0.25f);
	//nh.param("~wp_wt", wp_wait_time, 0.0f);

	bool ok1 = ros::param::get("~wp_rad", wp_radius) ;
	if (!ok1)
	{
		wp_radius = 0.25f;
	}

	bool ok2 = ros::param::get("~wp_wt", wp_wait_time) ;
	if (!ok2)
	{
		wp_wait_time = 0.0f;
	}

	ROS_INFO("WP Radius: %f\n", wp_radius);
	ROS_INFO("WP Wait Time: %f\n", wp_wait_time);

	sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav100/mavros/setpoint_position/local", 10);

	geometry_msgs::PoseStamped poseInit;
	poseInit.pose.position.x = 0.0f;
	poseInit.pose.position.y = 0.0f;
	poseInit.pose.position.z = 0.0f;
	poseInit.pose.orientation.x = 0.0f;
	poseInit.pose.orientation.y = 0.0f;
	poseInit.pose.orientation.z = 0.0f;
	poseInit.pose.orientation.w = 1.0f;

	//send a few setpoints before starting
	ros::Rate rate(20.0);
    for(int i = 100; ros::ok() && i > 0; --i){
        sp_pub.publish(poseInit);
        ros::spinOnce();
        rate.sleep();
    }
	
	ros::Timer timer = nh.createTimer(ros::Duration(0.05), callbackPostSetpoints);
	
	ros::NodeHandle nh_p;
	ros::CallbackQueue callback_queue_p;
	nh_p.setCallbackQueue(&callback_queue_p);
	//arming_client = nh_p.serviceClient<mavros_msgs::CommandBool>("/uav100/mavros/cmd/arming");
	mode_client = nh_p.serviceClient<mavros_msgs::SetMode>("/uav100/mavros/set_mode");
	pos_sub = nh_p.subscribe<geometry_msgs::PoseStamped>("/uav100/mavros/local_position/pose",1, callbackPositionRead);
	std::thread spinner_thread_p([&callback_queue_p]() {
		ros::SingleThreadedSpinner spinner_p;
		spinner_p.spin(&callback_queue_p);
	});
	ros::spin();
	spinner_thread_p.join();

	if (wp_file != NULL) fclose(wp_file);
  	return 0;
}

