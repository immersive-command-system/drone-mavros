#!/usr/bin/env python
import rospy
import roslibpy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint

class TopicPublisher:

    def __init__(self, namespace, connection):
        self.namespace = namespace
        self.connection = connection
        self.services = []
        self.advertise_service('/mavros/set_mode','mavros_msgs/SetMode', self.set_mode, include_namespace=True)
        self.advertise_service('/mavros/mission/push','mavros_msgs/WaypointPush',
                               self.mission_waypoint_push, include_namespace=True)

    def get_topics(self):
        return rospy.get_published_topics(self.namespace)

    def advertise_service(self, service_name, service_type, handler, include_namespace=False):
        if include_namespace:
            service_name = self.namespace + service_name
        service = roslibpy.Service(self.connection,
                                   service_name,
                                   service_type)
        print(service.name)
        service.advertise(handler)
        self.services.append(service)

    def set_mode(self, request, response):
        rospy.loginfo(request)
        set_mode_service = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        set_mode_object = SetMode()
        set_mode_object.custom_mode = request.get('custom_mode')
        local_response = set_mode_service(set_mode_object)
        response['mode_sent'] = local_response.mode_sent
        return local_response.mode_sent

    def mission_waypoint_push(self, request, response):
        rospy.loginfo(request)
        mission_push_service = rospy.ServiceProxy(self.namespace + '/mavros/mission/push', WaypointPush)
        waypoint_push_object = WaypointPush()
        waypoints = []
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
            waypoints.append(new_waypoint)
        waypoint_push_object.waypoints = waypoints
        print(waypoints)
        waypoint_push_object.start_index = request.get('start_index')
        local_response = mission_push_service(waypoint_push_object)
        response['success'] = local_response.success
        response['wp_transfered'] = local_response.wp_transfered
        return local_response.success
