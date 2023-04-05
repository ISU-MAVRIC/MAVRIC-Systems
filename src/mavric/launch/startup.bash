#!/bin/bash
chmod 666 /dev/ttyS*
source /home/mavric/MAVRIC-Systems/devel/setup.bash
roslaunch mavric "$(cat /opt/mavric/launchfile)"
