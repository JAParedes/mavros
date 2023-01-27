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
#include <mavros_msgs/State.h>
#include <boost/bind.hpp>

#include <string>
#include <mutex>
#include <cmath>

std::mutex mtx;

static FILE *wp_file = NULL;

float sp_x = 0.0f;
float sp_y = 0.0f;
float sp_z = 0.0f;
float sp_yaw = 0.0f;

float x00 = 0.0f;
float y00 = 0.0f;
float z00 = 0.0f;
float yaw00 = 0.0f;

float wp_radius = 0.25f;
float wp_wait_time = 2.0f;

float vel_xyz_max = 2.0f;
float acc_xyz_max = 0.5f;

float vel_yaw_max = 30.0f;
float acc_yaw_max = 10.0f;

float sp_rate = 0.05f;

const float m_pi = 3.14159265358979323846f;
const float deg2rad = m_pi/180.0f;

ros::Publisher sp_pub;
ros::Subscriber pos_sub;
ros::ServiceClient mode_client;

//mavros_msgs::State current_state;
//void state_cb(const mavros_msgs::State::ConstPtr& msg){
//    current_state = *msg;
//}

class WP_Filter {

	private:
		float dT;
		float t;
		float x0;
		float xr;
		float d_xr;
		float d_xT1;
		float sigma;
		float v_max;
		float a_max;
		float T1;
		float T2;
		float T3;
		float v_max_bar;
		float T1_bar;
		bool trapzVel;
	public:
		WP_Filter(float sp_pub_rate, float x0_par, float xr_init, float v_max_par, float a_max_par){
			dT = sp_pub_rate;
			t = 0.0f;
			x0 =  x0_par;
			xr = xr_init;
			d_xr = xr - x0;
			//sigma = sign(d_xr);
			sigma = (d_xr > 0.0f) - (d_xr < 0.0f);
			v_max = v_max_par;
			a_max = a_max_par;
			T1 = v_max / a_max;
			T2 = T1 + std::fabs(d_xr)/v_max - v_max/a_max;
			T3 = T1 + T2;
			d_xT1 = 0.5*a_max*T1*T1;
			v_max_bar = std::sqrt(std::fabs(d_xr)*a_max);
			T1_bar = std::sqrt(std::fabs(d_xr)/a_max);
			trapzVel = (std::fabs(d_xr) > 2.0f*std::fabs(d_xT1));
		}
		void newSP(float xr_n){
			x0 = xr;
			t = 0.0f;
			xr = xr_n;
			d_xr = xr - x0;
			sigma = (d_xr > 0.0f) - (d_xr < 0.0f);
			T2 = T1 + std::fabs(d_xr)/v_max - v_max/a_max;
			T3 = T1 + T2;
			v_max_bar = std::sqrt(std::fabs(d_xr)*a_max);
			T1_bar = std::sqrt(std::fabs(d_xr)/a_max);
			trapzVel = (std::fabs(d_xr) > 2.0f*std::fabs(d_xT1));

		}
		float run_filt(float xr_commanded){
			if (std::fabs(xr_commanded - xr) > 0.01f)
			{
				this->newSP(xr_commanded);
			}
			t += dT;
			float rk = 0.0f;
			if (trapzVel)
			{
				if (t < T1)
				{
					rk = sigma*(0.5*a_max*t*t);
				}
				else if ((T1 <= t) && (t < T2))
				{
					rk = sigma*(0.5*a_max*T1*T1 + v_max*(t - T1));
				}
				else if ((T2 <= t) && (t < T3))
				{
					rk = sigma*(0.5*a_max*T1*T1 + v_max*(T2 - T1)
					+ v_max*(t - T2) - 0.5*a_max*(t - T2)*(t - T2));
				}
				else
				{
					rk = d_xr;
				}
			}else{
				if (t < T1_bar)
				{
					rk = sigma*(0.5*a_max*t*t);
				}
				else if ((T1_bar <= t) && (t <= 2*T1_bar))
				{
					rk = sigma*(0.5*a_max*T1_bar*T1_bar + v_max_bar*(t - T1_bar)
					 - 0.5*a_max*(t - T1_bar)*(t - T1_bar));
				}
				else
				{
					rk = d_xr;
				}
			}
			rk += x0;
			return rk;
		}
};

void callbackPositionRead(ros::NodeHandle &node_handle, const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped poseTemp = *msg;
	static float spp_x = 0.0f;
	static float spp_y = 0.0f;
	static float spp_z = 0.0f;
	static float spp_yaw = 0.0f;
	static int count = 0;
	static bool start = false;

	boost::shared_ptr<mavros_msgs::State const> sharedMode0;
	mavros_msgs::State mode0;
	
	if (count == 0)
	{
		mtx.lock();
		sp_x = spp_x + x00;
		sp_y = spp_y + y00;
		sp_z = spp_z + z00;
		sp_yaw = deg2rad*spp_yaw + yaw00;
		mtx.unlock();
		count++;
	}

	if (!start){
		sharedMode0 = ros::topic::waitForMessage<mavros_msgs::State>("/uav100/mavros/state",node_handle);
		if(sharedMode0 != NULL){
			//ROS_INFO("Entered... \n");
			mode0 = *sharedMode0;
			if (mode0.mode == "OFFBOARD" && mode0.armed){
				start = true;
				ROS_INFO("Starting... \n");
				}
		}
	}

	float err_x_sq = (poseTemp.pose.position.y - (spp_x + x00))*(poseTemp.pose.position.y - (spp_x + x00));
	float err_y_sq = (poseTemp.pose.position.x - (spp_y + y00))*(poseTemp.pose.position.x - (spp_y + y00));
	float err_z_sq = (-poseTemp.pose.position.z - (spp_z + z00))*(-poseTemp.pose.position.z - (spp_z + z00));
	tf::Quaternion q(
			poseTemp.pose.orientation.y,
			poseTemp.pose.orientation.x,
			-poseTemp.pose.orientation.z,
			poseTemp.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	if (start){
		ROS_INFO("x_e: %f, y_e: %f, z_e: %f, yaw_e: %f \n", poseTemp.pose.position.y - (spp_x + x00), 
			poseTemp.pose.position.x- (spp_y + y00), -poseTemp.pose.position.z - (spp_z + z00), yaw - (deg2rad*spp_yaw + yaw00));
	}
	if (((err_x_sq + err_y_sq + err_z_sq)<=(wp_radius*wp_radius)) && (start))
	{
		if(fscanf(wp_file, "%f,%f,%f,%f", &spp_x, &spp_y, &spp_z, &spp_yaw) == EOF){
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
			mtx.lock();
			sp_x = spp_x + x00;
			sp_y = spp_y + y00;
			sp_z = spp_z + z00;
			sp_yaw = deg2rad*spp_yaw + yaw00;
			mtx.unlock();
		}
	}
}

void callbackPostSetpoints(const ros::TimerEvent& event)
{
	static geometry_msgs::PoseStamped pose;
    static int count = 0;
	static float spp2_x = x00;
	static float spp2_y = y00;
	static float spp2_z = z00;
	static float spp2_yaw = yaw00;
	static WP_Filter filt_x(sp_rate, spp2_x, spp2_x, vel_xyz_max, acc_xyz_max);
	static WP_Filter filt_y(sp_rate, spp2_y, spp2_y, vel_xyz_max, acc_xyz_max);
	static WP_Filter filt_z(sp_rate, spp2_z, spp2_z, vel_xyz_max, acc_xyz_max);
	static WP_Filter filt_yaw(sp_rate, spp2_yaw, spp2_yaw, vel_yaw_max, acc_yaw_max);
	
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

	ros::NodeHandle nh;
	ros::NodeHandle nh_p;

	//ros::Subscriber state_sub = nh_p.subscribe<mavros_msgs::State>
    //        ("mavros/state", 10, state_cb);

	ros::Duration(5).sleep();

	ROS_INFO("Setpoint publisher online!\n");

	bool ok1 = ros::param::get("~wp_rad", wp_radius) ;
	if (!ok1){wp_radius = 0.25f;}

	bool ok2 = ros::param::get("~wp_wt", wp_wait_time) ;
	if (!ok2){wp_wait_time = 2.0f;}
	
	bool ok3 = ros::param::get("~vel_xyz_max", vel_xyz_max) ;
	if (!ok3){vel_xyz_max = 2.0f;}

	bool ok4 = ros::param::get("~acc_xyz_max", acc_xyz_max) ;
	if (!ok4){acc_xyz_max = 0.5f;}

	bool ok5 = ros::param::get("~vel_yaw_max", vel_yaw_max) ;
	if (!ok5){vel_yaw_max = 30.0f;}

	bool ok6 = ros::param::get("~acc_yaw_max", acc_yaw_max) ;
	if (!ok6){acc_yaw_max = 10.0f;}

	bool ok7 = ros::param::get("~sp_rate", sp_rate) ;
	if (!ok7){sp_rate = 0.05f;}

	std::string wp_str;
	bool ok8 = ros::param::get("~wp_str", wp_str) ;
	if (!ok8)
	{
		wp_file = fopen("/home/umich-aero-2020/catkin_mavros_ws/waypoints.txt","r");
	}else{
		wp_file = fopen(wp_str.c_str(),"r");
	}

	ROS_INFO("WP Radius: %f\n", wp_radius);
	ROS_INFO("WP Wait Time: %f\n", wp_wait_time);
	ROS_INFO("Max vel XYZ: %f m/s\n", vel_xyz_max);
	ROS_INFO("Max acc XYZ: %f m/s^2\n", acc_xyz_max);
	ROS_INFO("Max vel yaw: %f deg/s\n", vel_yaw_max);
	ROS_INFO("Max acc yaw: %f deg/s^2\n", acc_yaw_max);
	ROS_INFO("Setpoint rate: %f sec\n", sp_rate);

	sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav100/mavros/setpoint_position/local", 10);

	boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPose0;
	geometry_msgs::PoseStamped pose0;
	sharedPose0 = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/uav100/mavros/local_position/pose",nh_p);
	if(sharedPose0 != NULL){
		pose0 = *sharedPose0;
	}

	x00 = pose0.pose.position.y;
	y00 = pose0.pose.position.x;
	z00 = -pose0.pose.position.z;
	tf::Quaternion q00(
		pose0.pose.orientation.y,
		pose0.pose.orientation.x,
		-pose0.pose.orientation.z,
		pose0.pose.orientation.w);
	tf::Matrix3x3 m00(q00);
	double roll00, pitch00, yaw000;
	m00.getRPY(roll00, pitch00, yaw000);
	yaw00 = yaw000;

	geometry_msgs::PoseStamped poseInit;
	poseInit.pose.position.x = x00;
	poseInit.pose.position.y = y00;
	poseInit.pose.position.z = z00;
	poseInit.pose.orientation.x = pose0.pose.orientation.y;
	poseInit.pose.orientation.y = pose0.pose.orientation.x;
	poseInit.pose.orientation.z = -pose0.pose.orientation.z;
	poseInit.pose.orientation.w = pose0.pose.orientation.w;

	//send a few setpoints before starting
	ros::Rate rate(20.0);
    for(int i = 100; ros::ok() && i > 0; --i){
        sp_pub.publish(poseInit);
        ros::spinOnce();
        rate.sleep();
    }
	
	ros::Timer timer = nh.createTimer(ros::Duration(sp_rate), callbackPostSetpoints);
	
	
	ros::CallbackQueue callback_queue_p;
	nh_p.setCallbackQueue(&callback_queue_p);
	//arming_client = nh_p.serviceClient<mavros_msgs::CommandBool>("/uav100/mavros/cmd/arming");
	mode_client = nh_p.serviceClient<mavros_msgs::SetMode>("/uav100/mavros/set_mode");
	pos_sub = nh_p.subscribe<geometry_msgs::PoseStamped>("/uav100/mavros/local_position/pose",1,
		boost::bind(&callbackPositionRead, boost::ref(nh_p), _1));
	std::thread spinner_thread_p([&callback_queue_p]() {
		ros::SingleThreadedSpinner spinner_p;
		spinner_p.spin(&callback_queue_p);
	});
	ros::spin();
	spinner_thread_p.join();

	if (wp_file != NULL) fclose(wp_file);
  	return 0;
}

