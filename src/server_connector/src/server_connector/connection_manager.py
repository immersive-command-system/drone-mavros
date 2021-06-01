#!/usr/bin/env python
import roslibpy
import rospy
import roslaunch
import os
from topic_publisher import TopicPublisher
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.msg import State

class ConnectionManager:

    instance = None

    def __init__(self, ip, port, name, fcu_url):
        self.ip = ip
        self.port = port
        self.name = name
        self.namespace = '/'
        self.server_connection = None
        self.topic_publisher = None
        self.persist_connection = True
        self.launched_mavros = False
        self.drone_state = None
        self.fcu_url = fcu_url
        ConnectionManager.instance = self
        rospy.Service('shutdown', Trigger, self.shutdown_service_callback)

    def register_drone(self, client, drone_name):
        service = roslibpy.Service(client, '/isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': drone_name, 'drone_type': 'Mavros'})

        result = service.call(request)
        rospy.loginfo(result)
        return result

    def launch_mavros(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        print(dir_path)
        new_path = os.path.join(dir_path, '../../launch/drone.launch')
        rospy.loginfo(new_path)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_file = [(new_path,
                           ['namespace:=' + self.namespace,
                            'fcu_url:=' + self.fcu_url])]
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

                # Starts Listening in For The Drone State (To Determine if Shutdowns are OK)
                rospy.Subscriber(self.namespace + '/mavros/state', State, self.state_subscriber_callback)

    def state_subscriber_callback(self, data):
        self.drone_state = data

    def check_connection(self):
        if self.persist_connection:
            if not self.server_connection.is_connected:
                self.attempt_connection()

    def stop_connection(self):
        self.persist_connection = False
        self.topic_publisher.unpublish()
        self.server_connection.terminate()

    def stop_connection_event(self, event):
        self.stop_connection()

    def shutdown_service_callback(self, req):
        success = False
        message = 'Unable to shutdown drone. May be armed.'
        if self.drone_state and not self.drone_state.armed:
            success = True
            message = "Shutting down drone."
            rospy.Timer(rospy.Duration(1.5), self.stop_connection_event, oneshot=True)
        return TriggerResponse(success, message)