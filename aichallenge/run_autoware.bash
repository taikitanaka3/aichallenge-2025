#!/bin/bash

mode="${1}"
id="${2:-0}" # デフォルト値0を設定

case "${mode}" in
"awsim")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=true")
    ;;
"awsim-no-viz")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=false")
    ;;
"vehicle")
    opts=("simulation:=false" "use_sim_time:=false" "run_rviz:=false")
    ;;
"rosbag")
    opts=("simulation:=false" "use_sim_time:=true" "run_rviz:=true")
    ;;
*)
    echo "invalid argument (use 'awsim' or 'vehicle' or 'rosbag')"
    exit 1
    ;;
esac

# idが0でない場合
if [ "$id" -ne 0 ]; then
    # Use loopback only and set domain for isolated runs
    export ROS_LOCALHOST_ONLY=0
    export ROS_DOMAIN_ID=$id
fi

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

ros2 launch aichallenge_system_launch aichallenge_system.launch.xml "${opts[@]}" "domain_id:=$id"
