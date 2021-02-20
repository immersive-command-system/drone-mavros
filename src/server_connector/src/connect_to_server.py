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
    new_path = os.path.join(dir_path, '../../../launch/drone.launch')
    rospy.loginfo(new_path)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    roslaunch_file = [(new_path,
                       ['namespace:=' + namespace])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()

def publish_topics(server_connection, id):
    service = roslibpy.Service(server_connection, '/isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
    request = roslibpy.ServiceRequest({"publishes": [{"name":namespace + "/mavros/set_mode",
                                                      "type": "mavros_msgs/SetMode"}], "id": id})
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
    result = register_drone(server_connection, 'hexacopter1')
    namespace = "/drone_" + str(id)
    rospy.loginfo('Connected! Launching MAVRos!')
    launch_mavros()
    publish_topics(server_connection, result['id'])

r = rospy.Rate(10)
rospy.loginfo(topic_publisher.get_topics(namespace))
while not rospy.is_shutdown():
    rospy.loginfo(server_connection.get_services())
    r.sleep()