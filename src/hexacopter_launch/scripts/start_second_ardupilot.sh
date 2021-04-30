#!/bin/bash
cd '/home/karthikdharmarajan/Documents/ardupilot/Tools/autotest' 
python sim_vehicle.py -L RFS -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I1
