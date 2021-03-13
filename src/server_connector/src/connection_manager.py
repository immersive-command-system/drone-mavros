#!/usr/bin/env python
import roslibpy
import rospy
import roslaunch
import os
from topic_publisher import TopicPublisher

class ConnectionManager:

    def __init__(self, ip, port, name):
        self.ip = ip
        self.port = port
        self.name = name
        self.namespace = '/'
        self.server_connection = None
        self.topic_publisher = None

    def register_drone(self, client, drone_name):
        service = roslibpy.Service(client, '/isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': drone_name, 'drone_type': 'Mavros'})

        result = service.call(request)
        rospy.loginfo(result)
        return result

    def launch_mavros(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        print(dir_path)
        new_path = os.path.join(dir_path, '../launch/drone.launch')
        rospy.loginfo(new_path)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_file = [(new_path,
                           ['namespace:=' + self.namespace])]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()

    def attempt_connection(self):
        self.server_connection = roslibpy.Ros(host=self.ip, port=self.port)
        rospy.loginfo('Attempting to connect to: ' + self.ip)
        self.server_connection.run(timeout=10)
        if self.namespace == '/':
            if self.server_connection.is_connected:
                result = self.register_drone(self.server_connection, self.name)
                self.namespace = "/drone_" + str(result['id'])
                rospy.loginfo('Connected! Launching MAVRos!')
                self.launch_mavros()
                self.topic_publisher = TopicPublisher(self.namespace, self.server_connection)

    def stop_connection(self):
        self.topic_publisher.unpublish()
        self.server_connection.terminate()