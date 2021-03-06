#!/usr/bin/env python
import roslibpy
import rospy
import os
import roslaunch
import topic_publisher

rospy.init_node('server_connector')
namespace = '/'

def register_drone(client, drone_name):
    service = roslibpy.Service(client, '/isaacs_server/register_drone', 'isaacs_server/register_drone')
    request = roslibpy.ServiceRequest({'drone_name': drone_name, 'drone_type': 'Mavros'})

    result = service.call(request)
    rospy.loginfo(result)
    return result


def launch_mavros():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)
    new_path = os.path.join(dir_path, '../launch/drone.launch')
    rospy.loginfo(new_path)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    roslaunch_file = [(new_path,
                       ['namespace:=' + namespace])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()

rospy.loginfo('Starting!')

server_ip = rospy.get_param('server_ip', '192.168.254.37')
server_port = 9090
server_connection = roslibpy.Ros(host=server_ip, port=server_port)
server_connection.run(timeout=10)
rospy.loginfo('Attempting to connect to: ' + server_ip)
result = None
if server_connection.is_connected:
    result = register_drone(server_connection, 'hexacopter2')
    namespace = "/drone_" + str(result['id'])
    rospy.loginfo('Connected! Launching MAVRos!')
    # launch_mavros()
    topic_publisher = topic_publisher.TopicPublisher(namespace, server_connection)

r = rospy.Rate(20)
while not rospy.is_shutdown():
    r.sleep()

server_connection.terminate()