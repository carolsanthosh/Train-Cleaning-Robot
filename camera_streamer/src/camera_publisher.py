#!/usr/bin/python3

import cv2
import rospy
from sensor_msgs.msg import CompressedImage
#from tflite_detect import detect
def usb_camera_publisher():
    rospy.init_node('usb_camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('usb_camera/image/compressed', CompressedImage, queue_size=1)
    cam_index = rospy.get_param("~camera_index")
    cap = cv2.VideoCapture(cam_index)  # Change the index if necessary


    rate = rospy.Rate(30)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        #frame,score = detect(frame)
        if ret:
            # Compress the image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, jpeg_image = cv2.imencode('.jpg', frame, encode_param)
            # Publish the compressed image
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpeg'
            msg.data = jpeg_image.tobytes()
            image_pub.publish(msg)
            print("cam_pub")
            # Sleep to achieve the desired publishing rate
            rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        usb_camera_publisher()
    except rospy.ROSInterruptException:
        pass
