#!/bin/bash
sleep 5
rosservice call /ur_hardware_interface/dashboard/load_program "{filename: \"/programs/xinlong.urp\"}"
sleep 5
rosservice call /ur_hardware_interface/dashboard/play "{}"
sleep 2
rosservice call /ur_hardware_interface/set_speed_slider "{ speed_slider_fraction: 0.4 }"
