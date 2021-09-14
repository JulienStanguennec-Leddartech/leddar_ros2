
import sys
import os
import time

#Import ros2 py
import rclpy 
from rclpy.node import Node

#Import messages 
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

#Import parameters (to read parameters)
from rclpy.parameter import Parameter

import numpy as np
import leddar


def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.

    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


class LeddarSensor(Node):

    def __init__(self):
        super().__init__('leddar_sensor')

        #Declare point cloud publisher topic
        self.publisher = self.create_publisher(sensor_msgs.PointCloud2, 'scan_cloud', 10)
        
        #Declaire parameter for connection to leddar_sensor | Default values for pixell sensor (Ethernet)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('param1', '192.168.0.2'),
                ('device_type', 'Ethernet'),
                ('param3', 48630),
                ('param4', 0)
            ]
        )

        #Read parameters for connection to leddar_sensor
        param1 = str(self.get_parameter('param1').value)
        device_type = str(self.get_parameter('device_type').value)
        param3 = int(self.get_parameter('param3').value)
        param4 = int(self.get_parameter('param4').value)

        #Create the sensor
        self.dev = leddar.Device()

        dev_type = 0
        if(device_type != "not specified"):
            dev_type = leddar.device_types[device_type]

        if not self.dev.connect(param1, dev_type, param3, param4):
            err_msg = 'Error connecting to device type {0} with connection info {1}/{2}/{3}.'.format(device_type, param1, str(param3), str(param4))
            #rclpy.logerr(err_msg)
            raise RuntimeError(err_msg)

        self.get_logger().info('Connected to device type {0} with connection info {1}/{2}/{3}.'.format(device_type, param1, str(param3), str(param4)))
        
        dev_type_read = self.dev.get_property_value(leddar.property_ids["ID_DEVICE_TYPE"])
        dev_protocol = self.dev.get_property_value(leddar.property_ids["ID_DATA_SERVER_PROTOCOL"])

        #Get info from sensor
        self.get_logger().info(f'ID_DEVICE_TYPE: {dev_protocol}')
        self.get_logger().info(f'ID_DATA_SERVER_PROTOCOL: {dev_protocol}')

        #Set callback method
        self.dev.set_callback_echo(self.echoes_callback)

        #Set datamask to detections
        self.dev.set_data_mask(leddar.data_masks["DM_ECHOES"])

        #Optionnal : set the delay between two request to the sensor
        self.dev.set_data_thread_delay(10000)
        self.dev.start_data_thread()

    #Callback functions for the data thread
    def echoes_callback(self, echoes):
        
        #keep valid echoes only
        echoes['data'] = echoes['data'][np.bitwise_and(echoes['data']['flags'], 0x01).astype(np.bool)] 

        #extract data field
        indices, flags, distances, amplitudes, x, y, z = [echoes['data'][x] for x in ['indices', 'flags', 'distances', 'amplitudes', 'x', 'y', 'z']]
        
        #merge xyz into np array
        xyz = np.array([x,y,z])
        
        #convert xyz np array to sensors_msg.PointCloud2
        message = point_cloud(xyz.T, 'map')

        #publish PointCloud2
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)

    leddar_sensor = LeddarSensor()

    rclpy.spin(leddar_sensor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leddar_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()