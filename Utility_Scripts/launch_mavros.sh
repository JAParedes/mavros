SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
LAUNCH_FILE="uavWiFi.launch"
roslaunch $SCRIPT_DIR/$LAUNCH_FILE
