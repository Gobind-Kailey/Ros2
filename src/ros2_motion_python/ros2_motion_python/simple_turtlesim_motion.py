

# Import ROS2 python library
import rclpy 
# Now we are importing Node Class
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time



# The standard to use in ROS2 is object oriented programming

# We are inheriting from Node Class
class TurtlesimMover(Node): 

    LINEAR_SPEED = 0.3
    ANGULAR_SPEED = 0.0

    def __init__(self):
         # Initializing the Node class with the name 'talker_publisher'
        super().__init__('turtlesimMover')
        # We need to specifiy the type, the name of the topic and the buffer size 
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5 # seconds or 2 Hz, which means 2 messages per second
         # We are creating a timer, which will call a function every timer_period seconds
         # The callback function is a function that will execute when an event occurs
         # Will run when the new event is created, and that could be when a message is received or a timer is triggered
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0 
        self.t0=time.time()


    def timer_callback(self):
        msg = Twist()
        # msg.data = 'Hello World: %d' % self.counter
        msg.linear.x=TurtlesimMover.LINEAR_SPEED
        msg.linear.y=0.0
        msg.linear.z=0.0

        msg.angular.x=0.0
        msg.angular.y=0.0
        msg.angular.z=TurtlesimMover.ANGULAR_SPEED

        # This line below tells us the current time
        t1 = time.time()

        # The total time spent would be the current time - the initial time
        time_spent = t1-self.t0
        print('Time spent: ', time_spent)

        if time_spent > 5:
            msg.linear.x=0.0
            msg.angular.z=0.0
            
            self.publisher.publish(msg)
            print('The robot has stopped')

            rclpy.shutdown()

        
        self.publisher.publish(msg) # need to change to sedning a twist message
        self.counter=self.counter + 1
        # The purpose of the logger is to provide a way to output 
        # information about the state of the program
            
        # self.get_logger().info('Publishing: "%s"' % msg)

    
def main (args=None): 

    # The purpose of the rclpy.init() function is to initialize the ROS2 communication
    rclpy.init(args=args)   
    # We are creating an object of the TalkerPublisher class
    turtlesim_mover = TurtlesimMover()
    # This is what dispatches the callbacks, it will keep the node alive
    rclpy.spin(turtlesim_mover)

    # Destroy the node explicitly
    turtlesim_mover.destroy_node()
    # Shut down the ROS2 communication, will kill all the nodes and end the program
    rclpy.shutdown()

# This is the standard way to run a python script
if __name__ == '__main__':
    main()

