#!/usr/bin/env python
import rospy
import roslibpy
from mavros_msgs.srv import SetMode

class TopicPublisher:

    def __init__(self, namespace, connection):
        self.namespace = namespace
        self.connection = connection
        self.services = []
        self.advertise_service('/mavros/set_mode','mavros_msgs/SetMode', self.set_mode, include_namespace=True)

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
        set_mode_service = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        set_mode_object = SetMode()
        set_mode_object.custom_mode = request.get('custom_mode')
        local_response = set_mode_service(set_mode_object)
        response['mode_sent'] = local_response.mode_sent
        return local_response.mode_sent
