#!/bin/bash
cd catkin_ws
source ~/.bashrc
echo "source enviroment"
cd
#rosrun mavros mavsys rate --all 10
gnome-terminal --window -e 'bash -c "roslaunch mavros apm.launch;exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch yahboomcar_visual ar_track_csi.launch"' \
--tab -e 'bash -c "sleep 6; rosrun iq_gnc 124GPSvisionland.cpp;exec bash"' \

