#!/bin/bash
cd catkin_ws
source ~/.bashrc
echo "source enviroment"
sudo chmod 777 /dev/ttyACM0
cd

gnome-terminal --window -e 'bash -c "roslaunch mavros apm.launch;exec bash"' \
rosrun mavros mavsys rate --all 10
--tab -e 'bash -c "sleep 2; roslaunch yahboomcar_visual ar_track_csi.launch"' \
--tab -e 'bash -c "sleep 6; rosrun iq_gnc 124GPSvisionland.cpp;exec bash"' \

