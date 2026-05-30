import rclpy 
from rclpy.node import Node 
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer 
from tf2_ros.transform_listener import TransformListener 
import math
import transforms3d 
import numpy as np 

 

class MarkerTfListener(Node): 

    def __init__(self): 

        super().__init__('marker_tf_listener') 

        #Create a Transform Buffer 

        #The Buffer is used to store the transforms 
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self) 

 

        #Create a Timer 

        timer_period = 0.1 #seconds 
        self.timer = self.create_timer(timer_period, self.timer_cb) 
        self.get_logger().info("Marker TF Listener Node has been started") 

 

 

    #Timer Callback 

    def timer_cb(self): 

        parent_frame = 'base_link'  
        child_frame= 'marker_0'  

        try: 

            self.transformation = self.tf_buffer.lookup_transform( 

                parent_frame, 
                child_frame, 
                rclpy.time.Time()) 

        except TransformException as ex: 

            self.get_logger().info( 
                f'Could not transform {child_frame} to {parent_frame}: {ex}') 

            return 

        # Get the position of the marker in the base_link frame 

        x = self.transformation.transform.translation.x 
        y = self.transformation.transform.translation.y 
        self.get_logger().info(f"X coordinate of {child_frame} is {x:.2f} m") 
        self.get_logger().info(f"Y coordinate of {child_frame} is {y:.2f} m") 

        # ADD YOUR CODE HERE: 
        distance = math.hypot(x, y)
        angle = math.atan2(y, x)
        angle_deg = math.degrees(angle)

        self.get_logger().info(
            f"Distance to {child_frame} is {distance:.2f} m")
        self.get_logger().info(
            f"Angle to {child_frame} is {angle:.2f} rad ({angle_deg:.1f} deg)")

 

 

def main(args=None): 

    rclpy.init(args=args) 
    m_p=MarkerTfListener() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

     

if __name__ == '__main__': 
    main() 
