#!/usr/bin/env python
import rospy
from server_connector.connection_manager import ConnectionManager
from server_connector.connect_to_server import get_connection_manager
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
                                       'parent_drone_name': rospy.get_param('name', 'hexacopter')})

connection_manager = ConnectionManager.instance
print('Getting connection manager instance')
print(connection_manager)
print(get_connection_manager())
if connection_manager != None:
    connection_manager.topic_publisher.publish_topic('/camera/depth/points',
                                                     'sensor_msgs/PointCloud2', PointCloud2,
                                                     point_cloud_publisher, include_namespace=True)
    server_connection = connection_manager.server_connection
    register_camera(server_connection)
