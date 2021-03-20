#!/usr/bin/env python
import rospy
from connection_manager import ConnectionManager

rospy.init_node('server_connector')
rospy.loginfo('Starting!')

server_ip = rospy.get_param('server_ip', '54.161.15.175')
server_port = 9090
connection_manager = ConnectionManager(server_ip, server_port, 'hexacopter')
connection_manager.attempt_connection()

r = rospy.Rate(0.2)
while not rospy.is_shutdown():
    connection_manager.check_connection()
    r.sleep()

connection_manager.stop_connection()