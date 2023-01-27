#include <stdio.h>
#include <thread>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <string>
#include <mutex>
#include <cmath>

std::mutex mtx;

static FILE *wp_file = NULL;

float sp_x = 0.0f;
float sp_y = 0.0f;
float sp_z = 0.0f;
float sp_yaw = 0.0f;

float wp_radius = 0.25f;
float wp_wait_time = 2.0f;

float zeta_filt = 1.2f;
float wn_filt = 1.0f;

float sp_rate = 0.05f;

const float m_pi = 3.14159265358979323846f;
const float deg2rad = m_pi/180.0f;


ros::Publisher sp_pub;
ros::Subscriber pos_sub;
ros::ServiceClient mode_client;

class WP_Filter {

	private:
		float yk1;
		float yk2;
		float numDT;
		float denDT2;
		float denDT3;
		float dT;
	public:
		WP_Filter(float init_val, float zeta, float wn, float sp_pub_rate){
			yk1 = init_val;
			yk2 = init_val;
			float rP = std::exp(-zeta*wn);
			float sqrtT2 = 1.0f - zeta*zeta;
			if (sqrtT2 <= 0.0f) {sqrtT2 = -sqrtT2;}
			float thetaP = wn*std::sqrt(sqrtT2);
			denDT2 = -2.0f*rP*std::cos(thetaP);
			denDT3 = rP*rP;
			numDT = 1.0f + denDT2 + denDT3;
			dT = sp_pub_rate;
		}
		float run_filt(float u){
			float yk = numDT*u - denDT2*yk1 - denDT3*yk2;
			yk2 = yk1;
			yk1 = yk;
			return yk;
		}
};

void callbackPositionRead(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped poseTemp = *msg;
	static float spp_x = 0.0f;
	static float spp_y = 0.0f;
	static float spp_z = 0.0f;
	static float spp_yaw = 0.0f;
	static float x_0 = 0.0f;
	static float y_0 = 0.0f;
	static float z_0 = 0.0f;
	static float yaw_0 = 0.0f;
	static int count = 0;
	if (count == 0)
	{
		fscanf(wp_file, "%f,%f,%f,%f", &spp_x, &spp_y, &spp_z, &spp_yaw);
		x_0 = poseTemp.pose.position.y;
		y_0 = poseTemp.pose.position.x;
		z_0 = -poseTemp.pose.position.z;
		tf::Quaternion q_0(
			poseTemp.pose.orientation.y,
			poseTemp.pose.orientation.x,
			-poseTemp.pose.orientation.z,
			poseTemp.pose.orientation.w);
		tf::Matrix3x3 m_0(q_0);
		double roll_0, pitch_0, yaw_00;
		m_0.getRPY(roll_0, pitch_0, yaw_00);
		yaw_0 = yaw_00;
		mtx.lock();
		sp_x = spp_x + x_0;
		sp_y = spp_y + y_0;
		sp_z = spp_z + z_0;
		sp_yaw = deg2rad*spp_yaw + yaw_0;
		mtx.unlock();
		count++;
	}
	float err_x_sq = (poseTemp.pose.position.y - (spp_x + x_0))*(poseTemp.pose.position.y - (spp_x + x_0));
	float err_y_sq = (poseTemp.pose.position.x - (spp_y + y_0))*(poseTemp.pose.position.x - (spp_y + x_0));
	float err_z_sq = (-poseTemp.pose.position.z - (spp_z + z_0))*(-poseTemp.pose.position.z - (spp_z + x_0));
	tf::Quaternion q(
			poseTemp.pose.orientation.y,
			poseTemp.pose.orientation.x,
			-poseTemp.pose.orientation.z,
			poseTemp.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	ROS_INFO("x_e: %f, y_e: %f, z_e: %f, yaw_e: %f \n", poseTemp.pose.position.y - (spp_x + x_0), poseTemp.pose.position.x- (spp_y + y_0), -poseTemp.pose.position.z - (spp_z + z_0), yaw - (deg2rad*spp_yaw + yaw_0));
	if ((err_x_sq + err_y_sq + err_z_sq)<=(wp_radius*wp_radius))
	{
		if(fscanf(wp_file, "%f,%f,%f,%f", &spp_x, &spp_y, &spp_z, &spp_yaw) == EOF){
			mtx.lock();
			sp_z = z_0 + 0.05f;
			mtx.unlock();
			ROS_INFO("Landing... \n");
		}else{
			ROS_INFO("Arrived at waypoint! \n");
			if (wp_wait_time>=0.01f){
				ros::Duration(wp_wait_time).sleep();
			}
			ROS_INFO("Next waypoint... \n");
			mtx.lock();
			sp_x = spp_x + x_0;
			sp_y = spp_y + y_0;
			sp_z = spp_z + z_0;
			sp_yaw = deg2rad*spp_yaw + yaw_0;
			mtx.unlock();
		}
	}
}

void callbackPostSetpoints(const ros::TimerEvent& event)
{
	static geometry_msgs::PoseStamped pose;
    	static int count = 0;
	static float spp2_x = 0.0f;
	static float spp2_y = 0.0f;
	static float spp2_z = 0.0f;
	static float spp2_yaw = 0.0f;
	static WP_Filter filt_x(0.0f,zeta_filt, wn_filt, sp_rate);
	static WP_Filter filt_y(0.0f,zeta_filt, wn_filt, sp_rate);
	static WP_Filter filt_z(0.0f,zeta_filt, wn_filt, sp_rate);
	static WP_Filter filt_yaw(0.0f,zeta_filt, wn_filt, sp_rate);
	
	if (count == 0)
	{
		pose.header.stamp = ros::Time::now();		
		pose.pose.position.x = 0.0f;
		pose.pose.position.y = 0.0f;
		pose.pose.position.z = 0.0f;
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		count++;
	}
	mtx.lock();
	spp2_x = sp_x;
	spp2_y = sp_y;
	spp2_z = sp_z;
	spp2_yaw = sp_yaw;
	mtx.unlock();

	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = filt_y.run_filt(spp2_y);
	pose.pose.position.y = filt_x.run_filt(spp2_x);
	pose.pose.position.z = -filt_z.run_filt(spp2_z);
	tf::Quaternion spp2_q = tf::createQuaternionFromYaw(filt_yaw.run_filt(spp2_yaw));
	pose.pose.orientation.x = spp2_q.y();
	pose.pose.orientation.y = spp2_q.x();
	pose.pose.orientation.z = -spp2_q.z();
	pose.pose.orientation.w = spp2_q.w();

	sp_pub.publish(pose);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setpoint_publisher");
	ros::Time::init();

	ros::Duration(5).sleep();

	ROS_INFO("Setpoint publisher online!\n");

	ros::NodeHandle nh;

	bool ok1 = ros::param::get("~wp_rad", wp_radius) ;
	if (!ok1){wp_radius = 0.25f;}

	bool ok2 = ros::param::get("~wp_wt", wp_wait_time) ;
	if (!ok2){wp_wait_time = 2.0f;}
	
	bool ok3 = ros::param::get("~zeta_filt", zeta_filt) ;
	if (!ok3){zeta_filt = 1.2f;}

	bool ok4 = ros::param::get("~wn_filt", wn_filt) ;
	if (!ok4){wn_filt = 1.0f;}

	bool ok5 = ros::param::get("~sp_rate", sp_rate) ;
	if (!ok5){sp_rate = 0.05f;}

	std::string wp_str;
	bool ok6 = ros::param::get("~wp_str", wp_str) ;
	if (!ok6)
	{
		wp_file = fopen("/home/umich-aero-2020/catkin_mavros_ws/waypoints.txt","r");
	}else{
		wp_file = fopen(wp_str.c_str(),"r");
	}

	ROS_INFO("WP Radius: %f\n", wp_radius);
	ROS_INFO("WP Wait Time: %f\n", wp_wait_time);
	ROS_INFO("Filter zeta: %f\n", zeta_filt);
	ROS_INFO("Filter wn: %f\n", wn_filt);
	ROS_INFO("Setpoint rate: %f sec\n", sp_rate);

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
	
	ros::Timer timer = nh.createTimer(ros::Duration(sp_rate), callbackPostSetpoints);
	
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

