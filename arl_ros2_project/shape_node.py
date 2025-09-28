#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class shape_node(Node):
    def __init__(self):
        super().__init__('shape_node')                                                                  #initialize the shape_node 
        self.pub = self.create_publisher(String, 'chosen_shape', 10)                                    #make it publisher to topic chosen_node
        self.valid_shapes = ['flower', 'spiral', 'heart','infinity','butterfly', 'stop']                                    #put valid shapes in a list
        self.get_logger().info("ShapeNode started and it is Waiting for input")                         #print this message to know the node initialized 
        self.run()
    def run(self):
        while rclpy.ok():
            shape=input("enter one of the following shapes (flower, spiral, heart, infinity, butterfly, stop)").strip().lower()    #the run function to run a loop to get input from the user and call publish_shape function
            self.publish_shape(shape)
            if not shape:                                                                                       #raise warning if user enters invalid input
                self.get_logger().warn("Empty input! Please type a shape.")
                continue
            if shape not in self.valid_shapes:
                self.get_logger().warn(f"Invalid shape: {shape} enter one of the valid shapes")
                continue
    def publish_shape(self,shape):                                                                               #get the shape entered and publish it as a message
        msg=String()
        msg.data = shape
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {shape}")

def main(args=None):
    rclpy.init(args=args)
    node = shape_node()
    try:
        rclpy.spin(node)          #to make node run continously and then close it safely from keyboard     
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()













