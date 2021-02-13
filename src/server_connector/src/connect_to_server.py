#!/usr/bin/env python
import roslibpy
import rospy

rospy.init_node('server_connector')


def register_drone(client):
    service = roslibpy.Service(client, '/isaacs_server/register_drone', 'isaacs_server/register_drone')
    request = roslibpy.ServiceRequest({'drone_name': 'hexacopter', 'drone_type': 'Mavros'})

    result = service.call(request)
    rospy.loginfo(result)
    return result


rospy.loginfo('Starting!')

server_ip = rospy.get_param('server_ip', '54.161.15.175')
server_port = 9090
server_connection = roslibpy.Ros(host=server_ip, port=server_port)
server_connection.run(timeout=10)
rospy.loginfo('Attempting to connect to: ' + server_ip)
result = None
if server_connection.is_connected:
    result = register_drone(server_connection)
    rospy.loginfo('Connected!')

rospy.spin()