import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import cv2
from tf_lite_detect import detect
rospy.init_node("speed_controller")

#sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)
cap = cv2.VideoCapture(2)

while not rospy.is_shutdown():
	ret, image = cap.read()
	image2, score = detect(image)
	speed.linear.x = 0.0
	speed.angular.z = 0.1
	cv2.imshow('Waste Detection', image2)
	if score == True:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		speed.linear.x = -0.1
		speed.angular.z = 0.0
	# Exit on 'q' key press
	if cv2.waitKey(1) == ord('q'):
		break
	
	pub.publish(speed)
	r.sleep()    


# Release the capture and destroy all windows
cap.release()
cv2.destroyAllWindows()
