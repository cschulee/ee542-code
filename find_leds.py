# find_leds.py Find LEDs
# 2014-10-30

# The purpose of this script is to find the 
#  coordinates of Light Emitting Diodes in
#  camera frame

# Currently it only looks for green and stores
# an intermediate and an altered picture back
# to the current working directory, to show
# things are working

# Import necessary packages
import picamera
import cv2
import numpy as np

# Connect to pi camera and capture image
camera = picamera.PiCamera()
camera.capture('image.jpg')
camera.close()

# Load image into python Open Computer Vision object
img = cv2.imread('/home/pi/ee542-code/image.jpg')

# Define min and max color range
GREEN_MIN = np.array([25,100,200],np.uint8)
GREEN_MAX = np.array([80,220,255],np.uint8)

# Convert img to Hue, Saturation, Value format
hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

# Threshold the image - results in b&w graphic where
#  in-threshold pixels are white and out-of-threshold
#  pixels are black
img_threshed = cv2.inRange(hsv_img, GREEN_MIN, GREEN_MAX)

# Find the circles
circles = cv2.HoughCircles(img_threshed,cv2.cv.CV_HOUGH_GRADIENT,1,75,param1=100,param2=5,minRadius=0,maxRadius=25)

# Temporary - mark the circles
for i in circles[0,:]:
	cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
	cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)

# Temprary - save the files back to disk to view later
cv2.imwrite('img_threshed.jpg',img_threshed)
cv2.imwrite('marked_image.jpg',img)
