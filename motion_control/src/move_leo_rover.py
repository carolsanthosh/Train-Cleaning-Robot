#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, Twist
from math import atan2
import cv2
from tf_lite_detect import detect
rospy.init_node("speed_controller")
import RPi.GPIO as GPIO
from time import sleep

ledpin = 12                     # PWM pin connected to motors
GPIO.setwarnings(False)                 #disable warnings
GPIO.setmode(GPIO.BOARD)                #set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,1000)          #create PWM instance with frequency

pi_pwm.start(0)
#rospy.init_node('cleanmaster', anonymous=True)
#sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
image_pub = rospy.Publisher('usb_camera/image/compressed', CompressedImage, que>

speed = Twist()

r = rospy.Rate(30)
cap = cv2.VideoCapture(0)
x_center = cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2  # Getting the frame center
while not rospy.is_shutdown():
        ret, image = cap.read()
        frame, score = detect(image)
	
	#contrarotate by default if no waste is detected
        speed.linear.x = 0.0
        speed.angular.z = 0.7
        pi_pwm.ChangeDutyCycle(0)

        #cv2.imshow('Waste Detection', frame)
        print(frame.shape)
        print(score)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, jpeg_image = cv2.imencode('.jpg', frame, encode_param)
	
	# Publish the compressed image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = jpeg_image.tobytes()
        image_pub.publish(msg)
	
        # Assume first detection is the target
        object_name, object_score, xmin, ymin, xmax, ymax, center_b = detections[0]
        xb_center, yb_center = center_b #the cneter of bounding box

        if score == True:
		if xb_center < x_center:
		 # Turn left
            	speed.linear.x = 0.0
            	speed.angular.z = 0.7
            	print("Turning left")
		elif xb_center > x_center:
		 # Turn right
            	speed.linear.x = 0.0
            	speed.angular.z = -0.7
            	print("Turning right")
		else:
		# Go straight
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                speed.linear.x = -0.2
                speed.angular.z = 0.0
		print("Going straight")

                pi_pwm.ChangeDutyCycle(50)

        # Exit on 'q' key press
	if cv2.waitKey(1) == ord('q'):
                break

        pub.publish(speed)
        r.sleep()


# Release the capture and destroy all windows
cap.release()
cv2.destroyAllWindows()
