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
        self.persist_connection = True
        self.launched_mavros = False

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
        try:
            self.server_connection.run(timeout=5)
        except Exception:
            rospy.logerr('Unable to connect to server!')
        if not self.launched_mavros:
            if self.server_connection.is_connected:
                result = self.register_drone(self.server_connection, self.name)
                self.namespace = "/drone_" + str(result['id'])
                rospy.loginfo('Connected! Launching MAVRos!')
                self.launch_mavros()
                self.launched_mavros = True
                self.topic_publisher = TopicPublisher(self.namespace, self.server_connection)

    def check_connection(self):
        if self.persist_connection:
            if not self.server_connection.is_connected:
                self.attempt_connection()

    def stop_connection(self):
        self.persist_connection = False
        self.topic_publisher.unpublish()
        self.server_connection.terminate()