#!/usr/bin/python3

import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter

lblpath = '/home/pi/cleanbot/src/camera_streamer/models/labelmap.txt'
modelpath = '/home/pi/cleanbot/src/camera_streamer/models/detect_quant.tflite'
min_conf=0.5

class detector:
    def __init__(self,lblpath,modelpath):
        with open(lblpath, 'r') as f:
            labels = [line.strip() for line in f.readlines()]
        self.interpreter = Interpreter(model_path=modelpath)
        self.interpreter.allocate_tensors()

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.float_input = (self.input_details[0]['dtype'] == np.float32)

        self.input_mean = 127.5
        self.input_std = 127.5

    def detect(self,image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        imH, imW, _ = image.shape
        image_resized = cv2.resize(image_rgb, (self.width, self.height))
        input_data = np.expand_dims(image_resized, axis=0)

        if self.float_input:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        boxes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]  # Bounding box coordinates of detected objects
        classes = self.interpreter.get_tensor(self.output_details[3]['index'])[0]  # Class index of detected objects
        scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]  # Confidence of detected objects

        return boxes,classes,scores
