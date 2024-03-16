#!/usr/bin/python3

import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter

lblpath = '/home/pi/cleanbot/src/camera_streamer/models/labelmap.txt'
modelpath = '/home/pi/cleanbot/src/camera_streamer/models/detect_quant.tflite'
min_conf=0.5

with open(lblpath, 'r') as f:
    labels = [line.strip() for line in f.readlines()]
interpreter = Interpreter(model_path=modelpath)
interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

float_input = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

def detect(image):
	image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	imH, imW, _ = image.shape
	image_resized = cv2.resize(image_rgb, (width, height))
	input_data = np.expand_dims(image_resized, axis=0)


	if float_input:
		input_data = (np.float32(input_data) - input_mean) / input_std
	interpreter.set_tensor(input_details[0]['index'], input_data)
	interpreter.invoke()
	boxes = interpreter.get_tensor(output_details[1]['index'])[0]  # Bounding box coordinates of detected objects
	classes = interpreter.get_tensor(output_details[3]['index'])[0]  # Class index of detected objects
	scores = interpreter.get_tensor(output_details[0]['index'])[0]  # Confidence of detected objects

	return boxes,classes,scores
