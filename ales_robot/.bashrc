export GAZEBO_MODEL_PATH=
source /opt/ros/fuerte/setup.bash
source /home/thelegace/fuerte_workspace/setup.bash
source /usr/share/gazebo/setup.sh
source /usr/share/drcsim/setup.sh
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/thelegace/fuerte_workspace/ales_robot/ales_msgs/lib
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}/home/thelegace/fuerte_workspace/mrp1_robot
