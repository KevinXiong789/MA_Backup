# **************************subscribe min_distance position from topic and send them to InfluxDB********************
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import time
from influxdb_client import InfluxDBClient, Point
#from influxdb_client.client.write_api import SYNCHRONOUS

class InfluxdbPublisherNode(Node):

    def __init__(self):
        super().__init__('influxdb_upload_node')
        #create a sub to get min_distance, this topic is published by min_distance_get_pub.cpp of pointcloud_test pkg
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'minimum_distance_topic',
            self.minimum_distance_callback,
            10)
        #set necessary info to connect InfluxDBClient
        self.influxdb_client = InfluxDBClient(url="http://localhost:8086", \
                                              token="EZZyYWflA8jgJFT1J5TfTkTbgECQQzcIbEXvTDKwBVKntwRm4JyAEy3wzjzJE20i-i-8k9vFbIO1WDxsGNQSPw==", \
                                              org="PointCloud")
        self.influxdb_write_api = self.influxdb_client.write_api()

    def minimum_distance_callback(self, msg):
        #show reaction in terminal
        #self.get_logger().info('I heard minimum distance: {:.4f}'.format(msg.data))
        self.get_logger().info('I heard minimum distance: %.4f' %msg.data[0])
        self.get_logger().info('x: %.4f, y: %.4f, d: %.4f' %(msg.data[1], msg.data[2], msg.data[3]))

        
        #get the currectly timestamp in ms
        timestamp = int(time.time() * 1000)
        
        #create point structure, that will be writed in InfluxDB
        data_point = Point("minimum_distance") \
            .field("min_distance", msg.data[0]) \
            .field("x", msg.data[1]) \
            .field("y", msg.data[2]) \
            .field("d", msg.data[3]) \
            .time(timestamp,"ms") 
        
        #write this point in the bucket
        self.influxdb_write_api.write(bucket="min_distance_test", record=data_point)
        #self.get_logger().info('timestamp {}'.format(timestamp))
        



def main(args=None):
    rclpy.init(args=args)
    influxdb_publisher_node = InfluxdbPublisherNode()
    rclpy.spin(influxdb_publisher_node)
    #influxdb_publisher_node.influxdb_client.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
