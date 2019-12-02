import tensorflow.keras as keras

import socketio
import eventlet
import numpy as np
from flask import Flask
from tensorflow.keras.models import load_model
import base64
from io import BytesIO
from PIL import Image
from PIL import ImageDraw 
from PIL import ImageFont
import cv2
import tensorflow as tf
from datetime import datetime
import os
sio = socketio.Server()
 
app = Flask(__name__) #'__main__'
speed_limit = 10
def img_preprocess(img):
    img = img[60:140,:,:]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img,  (3, 3), 0)
    return img
 
 
@sio.on('telemetry')
def telemetry(sid, data):
    speed = float(data['speed'])
    image = Image.open(BytesIO(base64.b64decode(data['image'])))
    image = np.asarray(image)
    image = img_preprocess(image)
    image = np.array([image])
    steering_angle = float(model.predict(image))
    throttle = 1.0 - speed/speed_limit
    print('{} {} {}'.format(steering_angle, throttle, speed))
    send_control(steering_angle, throttle)

    image = Image.open(BytesIO(base64.b64decode(data['image'])))
    draw = ImageDraw.Draw(image)
    draw.text((0, 0),"Autonomous Mode",(255,255,255))
    draw.text((0, 20),"steering : {}".format(steering_angle),(255,255,255))
    draw.text((0, 30),"speed : {}".format(speed),(255,255,255))
    draw.text((0, 40),"throttle : {}".format(throttle),(255,255,255))
    timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
    image_filename = os.path.join('./record', timestamp)
    image.save('{}.jpg'.format(image_filename))
 
 
 
@sio.on('connect')
def connect(sid, environ):
    print('Connected')
    send_control(0, 0)
 
def send_control(steering_angle, throttle):
    sio.emit('steer', data = {
        'steering_angle': steering_angle.__str__(),
        'throttle': throttle.__str__()
    })
 
 
if __name__ == '__main__':
    model = load_model('default_model_res.h5',compile = False)
    app = socketio.Middleware(sio, app)
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)