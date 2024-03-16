#!/usr/bin/python3

from flask import Flask, render_template, Response
import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
#from tflite_detect import detect
app = Flask(__name__)

def image_callback(msg):
    global current_image
    np_arr = np.frombuffer(msg.data, np.uint8)
    current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    

def generate():
    global current_image
    while True:
        ret, jpeg = cv2.imencode('.jpg', current_image)
        frame = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

def bbox_callback(msg):
     global current_image, output_image
     # Extract bounding box values from the received message
     ymin,xmin,ymax,xmax,classes,scores = msg.data
     cv2.rectangle(current_image, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)
     # Draw label
     object_name = labels[int(classes[i])]  # Look up object name from "labels" array using class index
     label = '%s: %d%%' % (object_name, int(scores[i] * 100))  # Example: 'person: 72%'
     labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
     label_ymin = max(ymin, labelSize[1] + 10)  # Make sure not to draw label too close to top of window
     cv2.rectangle(current_image, (xmin, label_ymin - labelSize[1] - 10),(xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255),cv2.FILLED)  
     # Draw white box to put label text in
     cv2.putText(current_image, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),2)  # Draw label text

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    rospy.init_node('web_display_node', anonymous=True)
    current_image = np.zeros((480, 640, 3), dtype=np.uint8)

    rospy.Subscriber('usb_camera/image/compressed', CompressedImage, image_callback)
    #rospy.Subscriber('/bounding_box', Float32MultiArray, bbox_callback)
    app.run(host='0.0.0.0', port=5000, threaded=True)
