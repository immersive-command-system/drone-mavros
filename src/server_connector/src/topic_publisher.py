#!/usr/bin/env python
import rospy
import roslibpy
from mavros_msgs.srv import SetMode

class TopicPublisher:
    topic_to_type = {
        'set_mode': 'mavros_msgs/SetMode',
        'cmd/arming': 'mavros_msgs/CommandBool',
        'cmd/takeoff': 'mavros_msgs/CommandTOL',
        'mission/push': 'mavros_msgs/WaypointPush'
    }

    service_name_exceptions = {
        'arming': 'cmd/arming',
        'takeoff': 'cmd/takeoff',
        'push': 'mission/push'
    }

    def __init__(self, namespace, connection):
        self.namespace = namespace
        self.connection = connection
        self.services = []

    def get_topics(self):
        return rospy.get_published_topics(self.namespace)

    def global_service(self, handler):
        """
        This method is designed to be used as a decorator (@global_service)
        to advertise the handler method as a service via the ROS_master_connection.
        By default, the service can be found at `/mavros/[handler_name]`
        with a service type found in the topic_to_type dictionary.

        Exceptions for the handler name to service type mapping can be added
        to the exceptions dictionary.None

        parameter: handler(request, response) handles an incoming service request.
        returns: handler
        """
        service_name = self.service_name_exceptions.get(handler.__name__, handler.__name__)
        service = roslibpy.Service(self.connection, self.namespace +
                                   '/mavros/{}'.format(service_name), self.topic_to_type[service_name])
        print(service.name)
        service.advertise(handler)
        self.services.append(service)
        return handler

    @global_service
    def set_mode(self, request, response):
        set_mode_service = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        set_mode_object = SetMode()
        set_mode_object.custom_mode = request.get('custom_mode')
        local_response = set_mode_service(set_mode_object)
        response['mode_sent'] = local_response.mode_sent
        return local_response.mode_sent


