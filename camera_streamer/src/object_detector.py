#!/usr/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from tflite_detect import detector
import numpy as np 

class BoundingBoxExtractor:
    def __init__(self):
        rospy.init_node('bounding_box_extractor', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('usb_camera/image/compressed', CompressedImage, self.image_callback)
        self.bbox_pub = rospy.Publisher('/bounding_box', Float32MultiArray, queue_size=1)
        self.min_conf=0.5
        self.lblpath = rospy.get_param('~lbl_path')
        self.modelpath = rospy.get_param('~model_path')
        self.detect = detector(self.lblpath,self.modelpath)
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            imH, imW, _ = cv_image.shape
            boxes,classes,scores = self.detect.detect(cv_image)
            detections = []
            detected = False
            for i in range(len(scores)):
                detected = True
                if ((scores[i] > self.min_conf) and (scores[i] <=1.0)):
                    # Get bounding box coordinates and dra
                    # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                    ymin = int(max(1, (boxes[i][0] * imH)))
                    xmin = int(max(1, (boxes[i][1] * imW)))
                    ymax = int(min(imH, (boxes[i][2] * imH)))
                    xmax = int(min(imW, (boxes[i][3] * imW)))
                    # Publish bounding box coordinates
                    bbox_msg = Float32MultiArray(data=[ymin, xmin, ymax, xmax,classes[i],scores[i]])
                    self.bbox_pub.publish(bbox_msg)
                    print(detected)

        except CvBridgeError as e:
            rospy.logerr('Error converting image: %s', str(e))

if __name__ == '__main__':
    try:
        bbox_extractor = BoundingBoxExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
