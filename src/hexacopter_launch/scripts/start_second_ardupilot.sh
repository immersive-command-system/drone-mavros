#!/bin/bash
cd '/home/karthikdharmarajan/Documents/ardupilot/Tools/autotest' 
python sim_vehicle.py -L RFS_2 -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I1
