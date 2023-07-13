# import the necessary packages
from picamera.array import PiRGBArray     
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import cv2
import cv2 as cv
import numpy as np

#hardware work
GPIO.setmode(GPIO.BOARD)

GPIO_TRIGGER1 = 36      #Left ultrasonic sensor
GPIO_ECHO1 = 33

GPIO_TRIGGER2 = 38      #Front ultrasonic sensor
GPIO_ECHO2 = 35

GPIO_TRIGGER3 = 40      #Right ultrasonic sensor
GPIO_ECHO3 = 37

MOTOR1B = 16  #Left Motor
MOTOR1E = 10

MOTOR2B = 11  #Right Motor
MOTOR2E = 13

#LED_PIN = 13  #If it finds the ball, then it will light up the led

# Set pins as output and input
GPIO.setwarnings(False)
GPIO.setup(GPIO_TRIGGER1,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO1,GPIO.IN)      # Echo
GPIO.setup(GPIO_TRIGGER2,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO2,GPIO.IN)
GPIO.setup(GPIO_TRIGGER3,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO3,GPIO.IN)
#GPIO.setup(LED_PIN,GPIO.OUT)

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER1, False)
GPIO.output(GPIO_TRIGGER2, False)
GPIO.output(GPIO_TRIGGER3, False)

# Allow module to settle
def sonar(GPIO_TRIGGER,GPIO_ECHO):
      start = 0
      stop = 0
      # Set pins as output and input
      GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
      GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo
     
      # Set trigger to False (Low)
      GPIO.output(GPIO_TRIGGER, False)
     
      # Allow module to settle
      time.sleep(0.01)
           
      #while distance > 5:
      #Send 10us pulse to trigger
      GPIO.output(GPIO_TRIGGER, True)
      time.sleep(0.00001)
      GPIO.output(GPIO_TRIGGER, False)
      begin = time.time()
      while GPIO.input(GPIO_ECHO)==0 and time.time()<begin+0.05:
            start = time.time()
     
      while GPIO.input(GPIO_ECHO)==1 and time.time()<begin+0.1:
            stop = time.time()
     
      # Calculate pulse length
      elapsed = stop-start
      # Distance pulse travelled in that time is time
      # multiplied by the speed of sound (cm/s) and divide by 2 because distance is there and back
      distance = elapsed * 34300 / 2
      
      # round to a reasonable number of sig figs
      distance = round(distance, 2) 
     
      #print ("Distance : %.1f" % distance)
      # Reset GPIO settings
      return distance

GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(MOTOR1E, GPIO.OUT)

GPIO.setup(MOTOR2B, GPIO.OUT)
GPIO.setup(MOTOR2E, GPIO.OUT)

def forward():
      GPIO.output(MOTOR1B, True)
      GPIO.output(MOTOR1E, False)
      GPIO.output(MOTOR2B, True)
      GPIO.output(MOTOR2E, False)
     
def reverse():
      GPIO.output(MOTOR1B, False)
      GPIO.output(MOTOR1E, True)
      GPIO.output(MOTOR2B, False)
      GPIO.output(MOTOR2E, True)
     
def turnright():
      GPIO.output(MOTOR1B, False)
      GPIO.output(MOTOR1E, True)
      GPIO.output(MOTOR2B, True)
      GPIO.output(MOTOR2E, False)
     
def turnleft():
      GPIO.output(MOTOR1B, True)
      GPIO.output(MOTOR1E, False)
      GPIO.output(MOTOR2B, False)
      GPIO.output(MOTOR2E, True)

def stop():
      GPIO.output(MOTOR1E, False)
      GPIO.output(MOTOR1B, False)
      GPIO.output(MOTOR2E, False)
      GPIO.output(MOTOR2B, False)

#CAMERA CAPTURE
#initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (160, 128)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(160, 128))

# allow the camera to warmup
time.sleep(0.001)
 
# capture frames from the camera
for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
      #grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
      frame = image.array
      frame=cv2.flip(frame,1)
      global centre_x
      global centre_y
      centre_x=0.
      centre_y=0.
      hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask_red = segment_colour(frame)      
      loct,area = find_blob(mask_red)
      x,y,w,h = loct
     
      #distance coming from front ultrasonic sensor
      distanceC = sonar(GPIO_TRIGGER2,GPIO_ECHO2)
      #distance coming from right ultrasonic sensor
      distanceR = sonar(GPIO_TRIGGER3,GPIO_ECHO3)
      #distance coming from left ultrasonic sensor
      distanceL = sonar(GPIO_TRIGGER1,GPIO_ECHO1)
             
      if (area) < 10:
            found=0
      else:
            found=1
            simg2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
            centre_x=x+((w)/2)
            centre_y=y+((h)/2)
            cv2.circle(frame,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
            centre_x-=80
            centre_y=6--centre_y
            print (centre_x, centre_y)
      initial=800
      flag=0
      #GPIO.output(LED_PIN,GPIO.LOW)
      if(found==0):
            if flag==0:
                  turnleft()
                  time.sleep(0.05)
            else:
                  turnright()
                  time.sleep(0.05)
            stop()
            time.sleep(0.0125)
      
      elif(found==1):
            print("distanceR = ", distanceR, ", distanceC = ", distanceC, ", distanceL = ", distanceL)
            
            if(centre_x<0):
                  flag=0
                  print("right")
                  turnright()
                  time.sleep(0.1)
            if(centre_x>0):
                  flag=1
                  print("left")
                  turnleft()
                  time.sleep(0.1)
            
            stop()
            time.sleep(0.00625)
                  
            if(area < 1200): 
                  print("forward")
                  forward()
                  time.sleep(0.1)
                  #stop()
                  time.sleep(0.00625)  
      
      cv2.imshow("draw",frame)    
      rawCapture.truncate(0)  # clear the stream in preparation for the next frame
         
      if(cv2.waitKey(1) & 0xff == ord('q')):
            break

GPIO.cleanup() #free all the GPIO pins
