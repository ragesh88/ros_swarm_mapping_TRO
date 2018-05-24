#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot

# The number of robots.  This should match the number of robots in the world file.
n=3

# launch file with path
LAUNCH_FILE=../launch/multi_robot.launch

echo "<launch>" > $LAUNCH_FILE

echo "<node pkg=\"stage_ros\" type=\"stageros\" name=\"stageros\" args=\"../worlds/test.world\" respawn=\"false\"/>" >> $LAUNCH_FILE

echo "<node pkg=\"map_sharing_info_based_exploration\" type=\"neighbors_srv\" name=\"Neighbors_node\" args=\"-n $n\"/>" >> $LAUNCH_FILE

for ((i=0; i<n; i++)); do
	echo -e "\t<node pkg=\"map_sharing_info_based_exploration\" type=\"MI_Levy_walker_sim\" name=\"MI_wanderer_node_$i\" args=\"-id 0 -n $n\"/>"
done >> $LAUNCH_FILE


echo -e "</launch>" >> $LAUNCH_FILE

roslaunch $LAUNCH_FILE
