import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time 

class Camsubscriber(Node):
      def __init__(self):
         super().__init__('cam_subscriber')
         self.subscriber = self.create_subscription(Image, 'camera_msg', self.callback, 10)
         self.bridge = CvBridge()
         print("Converting ROS2 Image message to OpenCV iplimage...")
         time.sleep(2)
         print("Initializing display")
 
      def callback(self, data): 
         sub_frames = self.bridge.imgmsg_to_cv2(data)
         cv2.namedWindow("User camera")
         cv2.imshow("User camera", sub_frames)
         key = cv2.waitKey(1) #Set the delay to 1 ms
         if key == 27: # Press esc key to exit 
            cv2.destroyAllWindows()
                     
def main():
    rclpy.init()
    node = Camsubscriber()
    try:
       rclpy.spin(node)
    except KeyboardInterrupt:
       pass
 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



