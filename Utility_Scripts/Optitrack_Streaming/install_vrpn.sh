cd ../../..
# mkdir vrpn_client_ros
git clone https://github.com/ros-drivers/vrpn_client_ros.git
cd vrpn_client_ros
sudo apt-get update
rosdep install --from-paths .
cd ..
catkin_make_isolated
