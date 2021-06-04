#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import roslibpy
import base64

def register_camera(server_connection):
    service = roslibpy.Service(server_connection, '/isaacs_server/register_sensor', 'isaacs_server/register_sensor')
    request = roslibpy.ServiceRequest({'sensor_name': 'kinect',
                                       'sensor_type': 'Depth Camera',
                                       'parent_drone_name': rospy.get_param('name', 'hexacopter')})
    service.call(request)


def setup(connection_manager):

    def image_publisher(publisher, data):
        sent = {'data': base64.b64encode(data.data).decode('ascii'),
                'format': data.format,
                'header': convert_header(data.header)}
        publisher.publish(sent)

    def convert_header(header):
        stamp = connection_manager.server_connection.get_time()
        return roslibpy.Header(seq=header.seq, stamp=stamp, frame_id=header.frame_id)

    connection_manager.topic_publisher.publish_topic('/camera/color/image_raw/compressed',
                                                     'sensor_msgs/CompressedImage', CompressedImage,
                                                     image_publisher, include_namespace=False)
    server_connection = connection_manager.server_connection
    register_camera(server_connection)
