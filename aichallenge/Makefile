# make file inspired by https://roborovsky-racers.github.io/RoborovskyNote/
sim:
	@sudo ip link set multicast on lo
	@sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
	@echo "Start AWSIM"
	@${AWSIM_PATH}/AWSIM.x86_64 --timeout 86400.0 &

aw:
	@echo "Start Auwoware"
	@bash run_autoware.bash

mpc: start
	@echo "Start MPC"
	@ros2 launch multi_purpose_mpc_ros mpc_controller.launch.py

start:
	@echo "Start control"
	ros2 topic pub -t 10 /control/control_mode_request_topic std_msgs/msg/Bool '{data: true}' >/dev/null &

reset:
	@echo "Reset simulation"
	@ros2 topic pub --once /aichallenge/awsim/reset std_msgs/msg/Empty {} >/dev/null &
	@ros2 run multi_purpose_mpc_ros publish_initialpose


setup: sim aw
	@echo ""

all: sim aw mpc
	@echo ""