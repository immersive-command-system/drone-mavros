#!/usr/bin/env python
import rospy
import roslibpy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import WaypointClear
from std_srvs.srv import Trigger

class TopicPublisher:

    def __init__(self, namespace, connection):
        self.namespace = namespace
        self.connection = connection
        self.services = []
        self.topics = []
        self.location = 0
        self.advertise_basic_services_topics()

    def advertise_basic_services_topics(self):
        self.advertise_service('/mavros/set_mode', 'mavros_msgs/SetMode', self.set_mode, include_namespace=True)
        self.advertise_service('/mavros/mission/push', 'mavros_msgs/WaypointPush',
                               self.mission_waypoint_push, include_namespace=True)
        self.advertise_service('/mavros/cmd/arming', 'mavros_msgs/CommandBool',
                               self.arm, include_namespace=True)
        self.advertise_service('/mavros/cmd/takeoff', 'mavros_msgs/CommandTOL',
                               self.takeoff, include_namespace=True)
        self.advertise_service('/mavros/mission/clear', 'mavros_msgs/WaypointClear',
                               self.waypoint_clear, include_namespace=True)
        self.advertise_service('/mavros/cmd/land', 'mavros_msgs/CommandTOL',
                               self.land_drone, include_namespace=True)
        self.advertise_service('/shutdown', 'std_srvs/Trigger'
                               , self.shutdown, include_namespace=True)
        self.publish_topic('/mavros/global_position/global', 'sensor_msgs/NavSatFix', NavSatFix
                           , self.gps_publisher, include_namespace=True)

    def get_topics(self):
        return rospy.get_published_topics(self.namespace)

    def advertise_service(self, service_name, service_type, handler, include_namespace=False):
        if include_namespace:
            service_name = self.namespace + service_name
        service = roslibpy.Service(self.connection,
                                   service_name,
                                   service_type)
        service.advertise(handler)
        self.services.append(service)

    def publish_topic(self, topic_name, topic_type, topic_type_class, publish_function, include_namespace=False):
        if include_namespace:
            topic_name = self.namespace + topic_name
        publisher = roslibpy.Topic(self.connection, topic_name, topic_type)
        self.topics.append(publisher)

        def publish(data):
            publish_function(publisher, data)

        rospy.Subscriber(topic_name, topic_type_class, publish)

    # ALL PUBLISHERS
    def gps_publisher(self, publisher, data):
        publisher.publish({'latitude': data.latitude, 'longitude': data.longitude, 'altitude': data.altitude})

        # DONE FOR TAKEOFF LOCATION
        self.location = data

    # ALL SERVICE CALLBACKS
    def set_mode(self, request, response):
        rospy.loginfo(request)
        set_mode_service = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        local_response = set_mode_service(request.get('basic_mode'), request.get('custom_mode'))
        response['mode_sent'] = True
        rospy.loginfo(local_response.mode_sent)
        return True

    def mission_waypoint_push(self, request, response):
        rospy.loginfo(request)
        mission_push_service = rospy.ServiceProxy(self.namespace + '/mavros/mission/push', WaypointPush)
        waypoints = []
        for i in range(2):
            takeoff_waypoint = Waypoint()
            takeoff_waypoint.command = 22
            takeoff_waypoint.param1 = 0
            takeoff_waypoint.param2 = 0
            takeoff_waypoint.param3 = 0
            takeoff_waypoint.param4 = 0
            if self.location:
                takeoff_waypoint.x_lat = self.location.latitude
                takeoff_waypoint.y_long = self.location.longitude
                takeoff_waypoint.z_alt = 10
            takeoff_waypoint.is_current = True
            takeoff_waypoint.autocontinue = True
            takeoff_waypoint.frame = 3
            waypoints.append(takeoff_waypoint)
        for waypoint in request.get('waypoints'):
            new_waypoint = Waypoint()
            new_waypoint.command = waypoint.get('command')
            new_waypoint.is_current = waypoint.get('is_current')
            new_waypoint.autocontinue = waypoint.get('autocontinue')
            new_waypoint.param1 = waypoint.get('param1')
            new_waypoint.param2 = waypoint.get('param2')
            new_waypoint.param3 = waypoint.get('param3')
            new_waypoint.param4 = waypoint.get('param4')
            new_waypoint.x_lat = waypoint.get('x_lat')
            new_waypoint.y_long = waypoint.get('y_long')
            new_waypoint.z_alt = waypoint.get('z_alt')
            new_waypoint.frame = waypoint.get('frame')
            waypoints.append(new_waypoint)
        rospy.loginfo(waypoints)
        local_response = mission_push_service(request.get('start_index'), waypoints)
        response['success'] = local_response.success
        response['wp_transfered'] = local_response.wp_transfered
        return local_response.success

    def arm(self, request, response):
        rospy.loginfo(request)
        arm_service = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
        local_response = arm_service(request.get('value'))
        response['success'] = local_response.success
        rospy.loginfo(local_response.success)
        return True

    def takeoff(self, request, response):
        rospy.loginfo(request)
        takeoff_service = rospy.ServiceProxy(self.namespace + '/mavros/cmd/takeoff', CommandTOL)
        local_response = takeoff_service(request.get('min_pitch'), request.get('yaw'),
                                         request.get('latitude'), request.get('longitude'),
                                         request.get('altitude'))
        response['success'] = local_response.success
        rospy.loginfo(local_response.success)
        return True

    def waypoint_clear(self, request, response):
        rospy.loginfo(request)
        waypoint_clear_service = rospy.ServiceProxy(self.namespace + '/mavros/mission/clear', WaypointClear)
        local_response = waypoint_clear_service()
        response['success'] = local_response.success
        rospy.loginfo(local_response.success)
        return True

    def land_drone(self, request, response):
        rospy.loginfo(request)
        land_service = rospy.ServiceProxy(self.namespace + '/mavros/cmd/land', CommandTOL)
        local_response = land_service(request.get('min_pitch'), request.get('yaw'),
                                      request.get('latitude'), request.get('longitude'),
                                      request.get('altitude'))
        response['success'] = local_response.success
        rospy.loginfo(local_response.success)
        return True

    def shutdown(self, request, response):
        rospy.loginfo(request)
        shutdown_service = rospy.ServiceProxy('/shutdown', Trigger)
        shutdown_service_response = shutdown_service()
        response['success'] = shutdown_service_response.success
        response['message'] = shutdown_service_response.message
        return True

    def unpublish(self):
        for service in self.services:
            service.unadvertise()
        for topic in self.topics:
            topic.unadvertise()