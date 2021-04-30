#!/usr/bin/env python

from server_connector.connection_manager import ConnectionManager
from sensor_msgs.msg import PointCloud2
import roslibpy

def point_cloud_publisher(publisher, data):
    publisher.publish({'header': data.header,
                       'height': data.height,
                       'width': data.width,
                       'fields': data.fields,
                       'is_bigendian': data.is_bigendian,
                       'point_step': data.point_step,
                       'row_step': data.row_step,
                       'data': data.data,
                       'is_dense': data.is_dense})

def register_camera(server_connection):
    service = roslibpy.Service(server_connection, '/isaacs_server/register_sensor', 'isaacs_server/register_sensor')
    request = roslibpy.ServiceRequest({'sensor_name': 'kinect',
                                       'sensor_type': 'depth_camera',
                                       'parent_drone_name': 'Hexacopter'})

connection_manager.topic_publisher.publish_topic('/camera/depth/points',
                                                 'sensor_msgs/PointCloud2', PointCloud2,
                                                 point_cloud_publisher, include_namespace=True)
server_connection = connection_manager.server_connection
register_camera(server_connection)
