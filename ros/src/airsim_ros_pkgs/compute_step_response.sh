cd ~/Research/CAML/AirSim/ros/src/airsim_ros_pkgs/build && make

cd ~/Research/CAML/Blocks

./Blocks.sh -windowed -ResX=1080 -ResY=720 &

cd ~/Research/CAML/AirSim/ros/src/airsim_ros_pkgs

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)

sleep 4

roslaunch airsim_ros_pkgs airsim_node_car.launch &

sleep 3

rosbag record /airsim_node/PhysXCar/odom_local_ned /airsim_node/PhysXCar/cmd_vel --duration 95s -O step_response_data.bag &

rosbag play step-response-input-05-09.bag --clock -u 95

sleep 3

killall Blocks

sleep 3

killall roslaunch

sleep 3

python src/plot_step_response.py --bag_file step_response_data.bag
