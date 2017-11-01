#!/bin/bash
## Connect to the first drone
echo " Connecting to the first ardrone" 
nmcli d wifi connect ardrone2_004420
# status = nmcli d status{STATE}
# if status == 'connected'; then 
# if ["$Value" == 1]; then
echo "/data/wifi.sh" #| 
telnet 192.168.1.1
echo " First Drone connected"
nmcli dev disconnect iface wlan0
# else
# echo " Did not connect to the first drone"
# fi

## Connect to the second drone
echo " Connecting to the second ardrone"
nmcli d wifi connect ardrone2_307158
# read Value
# if ["$Value" == 1]; then
echo "/data/wifi.sh"  
telnet 192.168.1.1
echo "Second Drone connected"
nmcli dev disconnect iface wlan0
# else
	# echo " Did not connect to the first drone"
# fi
echo "Finished connecting two drones"
echo "Connecting to the other wifi router"
nmcli d wifi connect ACS_LAB_DRONES_2.4G
cd ~/ardrone_catkin_ws/
catkin_make
source devel/setup.bash
sleep 10
echo " Running Roslaunch from ardrone_autonomy for connecting two drones"
roslaunch ardrone_autonomy ardrone_group.launch
echo "Successfully launched group launch"
# echo "Subscribing to image topics from ardrone 1 and ardrone 2"
# rosrun image_view image_view image:=/ardrone1/ardrone/front/image_raw
# rosrun image_view image_view image:=/ardrone2/ardrone/front/image_raw

