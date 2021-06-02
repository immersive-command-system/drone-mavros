#!/usr/bin/env python
import rospy
import importlib
from connection_manager import ConnectionManager

if __name__ == '__main__':
	rospy.init_node('server_connector')
	rospy.loginfo('Starting!')

	server_ip = rospy.get_param('server_ip', '54.161.15.175')
	server_port = 9090
	name = rospy.get_param('name', 'hexacopter')
	fcu_url = rospy.get_param('fcu_url', 'udp://127.0.0.1:14551@14555')
	rospy.loginfo('Starting with name ' + name + ' to ip ' + server_ip + ' with fcu_url ' + fcu_url)
	connection_manager = ConnectionManager(server_ip, server_port, name, fcu_url)
	connection_manager.attempt_connection()

	setup_modules = rospy.get_param('setup_modules')
	for module in setup_modules:
		imported = importlib.import_module(module)
		imported.setup(connection_manager)

	r = rospy.Rate(0.2)
	while not rospy.is_shutdown():
		connection_manager.check_connection()
		r.sleep()

	connection_manager.stop_connection()
