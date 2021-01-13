#include <stdio.h>
#include <thread>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>

#include <mutex>

std::mutex mtx;

static FILE *wp_file = NULL;

float sp_x = 0.0f;
float sp_y = 0.0f;
float sp_z = 0.0f;

ros::Publisher sp_pub;
ros::Subscriber pos_sub;
ros::ServiceClient arming_client;

void callbackPositionRead(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped poseTemp = *msg;
	static float spp_x = 0.0f;
	static float spp_y = 0.0f;
	static float spp_z = 0.0f;
	static int count = 0;
	if (count == 0)
	{
		fscanf(wp_file, "%f,%f,%f", &spp_x, &spp_y, &spp_z);
		mtx.lock();
		sp_x = spp_x;
		sp_y = spp_y;
		sp_z = spp_z;
		mtx.unlock();
		count++;
	}
	float err_x_sq = (poseTemp.pose.position.x - spp_x)*(poseTemp.pose.position.x - spp_x);
	float err_y_sq = (poseTemp.pose.position.y - spp_y)*(poseTemp.pose.position.y - spp_y);
	float err_z_sq = (poseTemp.pose.position.z - spp_z)*(poseTemp.pose.position.z - spp_z);
	if ((err_x_sq + err_y_sq + err_z_sq)<0.25)
	{
		if(fscanf(wp_file, "%f,%f,%f", &spp_x, &spp_y, &spp_z) == EOF){
			mavros_msgs::CommandBool arm_cmd;
    		arm_cmd.request.value = false;
			arming_client.call(arm_cmd);
		}else{
			mtx.lock();
			sp_x = spp_x;
			sp_y = spp_y;
			sp_z = spp_z;
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
	mtx.unlock();
	sp_pub.publish(pose);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setpoint_publisher");
	ros::Time::init();

	ros::Duration(5).sleep();

	wp_file = fopen("/home/umich-aero-2020/catkin_mavros_ws/waypoints.txt","r");

	ROS_INFO("Setpoint publisher online!");

	ros::NodeHandle nh;
	sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav100/mavros/setpoint_position/local", 10);

	geometry_msgs::PoseStamped poseInit;
	poseInit.pose.position.x = 0.0f;
	poseInit.pose.position.y = 0.0f;
	poseInit.pose.position.z = 0.0f;

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
	arming_client = nh_p.serviceClient<mavros_msgs::CommandBool>("/uav100/mavros/cmd/arming");
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

