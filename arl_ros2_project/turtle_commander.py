#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
import math, time, threading, queue

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)                      #publish velocity to turtlesim_node
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)        #subscribe to get pose from turtlesim_node and pass thepose to the function
        self.shape_sub = self.create_subscription(String, 'chosen_shape', self.shape_cb, 10)     #subscribe to get shape from chosen shape and cmd_pub means it is a command publisher
        self.pose = None
        self.shape_q = queue.Queue()                                                             #make a queue to not interrupt shapes
        self.stop_event = threading.Event()   
        self.draw_thread = threading.Thread(target=self.draw_worker, daemon=True)                #start a thread to draw in the background while also taking orders and updating the pose         
        self.draw_thread.start()
        self.get_logger().info("TurtleCommander ready. Waiting for shapes...")

    def pose_cb(self, msg):
        self.pose = msg                                                                           #update the position according to input taken from turtlism node

    def shape_cb(self, msg):
        shape = msg.data                                                          #add the shapes comming from shape_node
        self.get_logger().info(f"Got shape: {shape}")
        self.shape_q.put(shape)

    def draw_worker(self):
        while rclpy.ok():
            try:
              shape = self.shape_q.get(timeout=0.1)
            except queue.Empty:
               continue
            if shape == 'stop':
                self.stop_turtle_now()
                continue  
            self.stop_event.clear()
            self.stop_shape = False                                      # reset before new shape input
            start = time.time()                                             
            while self.pose is None and time.time() - start < 5.0:        #wait for shape and raise warning if something is wrong
                time.sleep(0.05)
            if self.pose is None:
                self.get_logger().warn("No /turtle1/pose received - aborting shape.")
                continue

            if shape == 'flower':
                pts = self.make_flower()                                                        #pts = the points needed to draw the shape
            elif shape == 'spiral':
                pts = self.make_spiral()
            elif shape == 'heart':
                pts = self.make_heart()
            elif shape == 'infinity':
                pts = self.make_infinity()
            elif shape == 'butterfly':
                pts = self.make_butterfly()
            else:
                self.get_logger().info("Unknown shape. enter one of the following shapes (flower, spiral, heart, infinity, butterfly, stop)")        #already handeled in the shape_node file
                continue

            for (x,y) in pts:
                self.move_to(x, y)
                if self.stop_event.is_set():
                    self.get_logger().info("shape drawing stopped")
                    break

    def stop(self,log=True):                                                                                #make velocity zero and sends the message
        t = Twist()
        self.cmd_pub.publish(t)
        if log:
            self.get_logger().info("stopped turtle")

    def stop_turtle_now(self):
        self.get_logger().info("Turtle stopped immediately")
        self.stop_event.set()  
        t = Twist()
        self.cmd_pub.publish(t)  
        with self.shape_q.mutex:
            self.shape_q.queue.clear()  



    def move_to(self, x, y):
        rate_hz = 20.0                                                                    #update commands per second                                                        
        dt = 1.0 / rate_hz
        while rclpy.ok():
            if self.stop_event.is_set():
                t=Twist()
                self.cmd_pub.publish(t)
                return
            if self.pose is None:                                                         #keep checking pose every 0.01 s
                time.sleep(0.01)
                continue
            dx = x - self.pose.x
            dy = y - self.pose.y
            dist = math.hypot(dx, dy)                                                      #compute distance between current and go to point
            if dist < 0.05:                                                                 #to avoid jittering 
                self.stop(log=False)
                return
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.pose.theta)               #get angle difference between -+ pi

            twist = Twist()                                                                #turning priority to either turn first or slightly adjust rotation
            if abs(angle_diff) > 0.12:
                twist.angular.z = 3.0 * angle_diff                                        
                twist.linear.x = 0.0
            else:
                twist.angular.z = 2.0 * angle_diff
                twist.linear.x = min(1.5, 0.8 * dist)                                  #move with v = 0.8*dist but the maximum is 1.5
            self.cmd_pub.publish(twist)
            time.sleep(dt)

    def normalize_angle(self, a):                                                     #adjust angle difference to be between -+ pi
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a
    def make_flower(self):
        pts = []
        cx, cy = 5.5, 5.5
        k = 4      # number of petals
        scale = 3.0
        steps = 300
        for i in range(steps):
            t = 2 * math.pi * i / steps
            r = math.sin(k * t)
            x = cx + scale * r * math.cos(t)
            y = cy + scale * r * math.sin(t)
            pts.append((x, y))
        return pts
    def make_spiral(self):
        pts = []
        cx, cy = 5.5, 5.5
        steps = 300
        for i in range(steps):
            t = 0.1 * i
            r = 0.06 * i
            x = cx + r * math.cos(t)
            y = cy + r * math.sin(t)
            pts.append((x, y))
        return pts

    def make_heart(self):
        pts = []
        cx, cy = 5.5, 5.5
        steps = 300
        scale = 0.12
        for i in range(steps):
            t = 2*math.pi * (i/steps)
            x = 16 * (math.sin(t)**3)
            y = 13*math.cos(t) - 5*math.cos(2*t) - 2*math.cos(3*t) - math.cos(4*t)
            pts.append((cx + scale * x, cy + scale * y))
        return pts
    def make_infinity(self):
        pts = []
        cx, cy = 5.5, 5.5
        a = 3.0       # size of the loops
        steps = 300
        for i in range(steps):
            t = 2 * math.pi * i / steps
            x = cx + a * math.sin(t)
            y = cy + a * math.sin(t) * math.cos(t)
            pts.append((x, y))
        return pts
    def make_butterfly(self):
        pts = []
        cx, cy = 5.5, 5.5
        steps = 400
        scale = 1.5
        for i in range(steps):
            t = 2 * math.pi * i / steps
            r = math.exp(math.sin(t)) - 2 * math.cos(4*t) + math.sin((2*t - math.pi)/24)**5
            x = cx + scale * r * math.cos(t)
            y = cy + scale * r * math.sin(t)
            pts.append((x, y))
        return pts
def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


