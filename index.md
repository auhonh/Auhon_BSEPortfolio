# Computer Vision Robot
My project is a Computer Vision Robot. It is a three-wheeled robot that utilizes ultrasonic sensors to detect a red ball and both turn and move toward it. It does this using a Raspberry Pi 4 where code is loaded and run, determining when each of the motors should run depending on the output of the ultrasonic sensors. 

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Auhon H. | Homestead High | Mechanical Engineering | Incoming Junior

<!--
**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
-->

# Third Milestone - Finished Robot

<iframe width="560" height="315" src="https://www.youtube.com/embed/Z9Io0Sug-JA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For my third milestone, I coded the motors to respond to output from the camera and the ultrasonic sensors. If the ball is too far to the right or left, the robot will turn right or left respectively. If the ball is in front of the robot and a certain distance away, the robot will move forward. However, this only works when the ball is in the field of vision of the camera, so in order to locate the ball when it rolls out of view, the robot rotates in the direction that it last saw the ball. 

This works using the camera and the three ultrasonic sensors. Using the front camera, it detects the location of the ball relative to the center of the camera. If the ball is too far to the left, the motors will turn the robot left, and vice versa when the ball is too far to the right. Additionally, the program calculates the area of the bounding box to get a general distance for how far the ball is, and it uses this in combination with all three ultrasonic sensors to determine when to move forward or stop. For example, if the area of the bounding box is small enough and the distance on the center sensor is large enough, the robot will move forward, along with many other combinations of output from the camera and sensors.

<!--Initially, this did not work, so I began to troubleshoot. I started with -->


# Second Milestone - Object Tracking Finished
<iframe width="560" height="315" src="https://www.youtube.com/embed/smtitpBq4H8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For my second milestone, I finished all the code related to tracking objects, in this case, a red ball. This code is made possible by the Raspberry Pi camera module and the three ultrasonic sensors. The first thing I did was get all of the camera-related code working, which consists of returning input, and locating the center of the red ball. For the ultrasonic sensors, I intended to find the distance that the nearest object was from each sensor. 

Before coding anything, I had to get Raspberry Pi OS loaded onto the Pi and find a way to code on it without a monitor. To install its OS, I simply inserted a microSD card into my computer and used the Raspberry Pi website to load the OS onto the card. Following this, I removed the card from my computer and moved it into the Pi itself. However, I still had to find a way to code on it, so I plugged the Pi into my computer with a capture card. After this, I used a program primarily used for Twitch streaming, OBS, to screencast the Raspberry Pi to my laptop, allowing me to code on it.

Now that I could code, getting the camera to return input was relatively easy because of its direct connection to the Raspberry Pi through a ribbon cable. All I had to do was make a variable to store the camera, which I could use to access what it sees. The tricky part was getting the camera to track the red ball. I did this by taking each frame, eroding and diluting the colors until all colors that were not a specific shade of red were set to black, and making that shade of red white. Once I had this done, I located any white oval and created a bounding box around it. Once I had done this, I used simple geometry to locate the center of this rectangle, which allows me to find the center of the red ball. 

Once the camera-related code was finished, I needed to find how close each ultrasonic sensor was to the nearest object. There are a total of three sensors, one facing left, one facing right, and one facing forward. To begin, I powered each sensor by plugging their VCC and Ground pins into a breadboard, which I powered using pins on the Raspberry Pi. To get input, I plugged the Trigger and Echo pins directly into the Raspberry Pi. I then used these pins to find how long it took for a pulse to be sent forward and reflected back to the sensor. I then used math to convert the pulse length into centimeters and stored each of them in a variable, so I can refer to them later.

Now, these numbers mean nothing on their own, so I have to use the center of the rectangle and the distances to direct the motors when to run. For example, if the center of the ball is too far left or right, the motors need to turn in that direction, or if the ball is too far, then the robot needs to move towards it. Additionally, I still need to finish CADing and 3D printing the new chassis from my first milestone.

# First Milestone - All Wiring and Assembly Done
<iframe width="560" height="315" src="https://www.youtube.com/embed/GzlZ3yV8Udk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For my first milestone, I assembled and wired the robot. The components of my robot are divided into three sections: Movement, Core, and Visuals. For Movement, I have two generic DC motors, two motor-powered wheels, a swiveling wheel, a plastic chassis, and an L298 H-Bridge motor driver. The Core section contains two battery packs, a switch, a generic breadboard, and a Raspberry Pi 4. As for the Visuals section, it contains a Raspberry Pi camera module and three Arduino ultrasonic sensors. 

Currently, the Movement section has two motor-powered wheels  mounted to the DC motors. These are in turn mounted to the back of the plastic chassis. The swivel wheel is attached to the front of the chassis with standoffs, and the H-Bridge is wired to the motors and the Raspberry Pi. For the Core section, there are two battery packs connected to the switch, which powers the H-bridge and Raspberry Pi when turned on. Additionally, the Raspberry Pi supplies power to the camera module and breadboard, the latter of which powers all of the ultrasonic sensors. The final section, Visuals, has the camera module connected to the Raspberry Pi with its ribbon cable, powering it. The ultrasonic sensors have their VCC and Ground wires plugged into the breadboard, so the Raspberry Pi can power them, and the Trigger and Echo wires connect directly to the Raspberry Pi, providing input. 

The current chassis is a generic plastic base, making it hard to mount all of the electrical components on the robot itself. As a result, I had to shorten and lengthen certain wires using wire strippers, a soldering iron, heat-shrink tubing, and more in order to mount them properly. Right now, I still need to write code that allows the Raspberry Pi to command the motors to turn on depending on inputs from the camera and ultrasonic sensors. I also need to CAD and 3D print a new chassis to better accommodate the various electronics.

<!--
# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resources to create professional schematic diagrams, though BSE recommends Tinkercad because it can be done easily and for free in the browser. 
-->


# Code

```python
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

MOTOR1B = 10  #Left Motor
MOTOR1E = 12

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
     
#Image analysis work
def segment_colour(frame):    #returns only the red colors in the frame
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    ycr_roi=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))

    mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)      #Eroding
    mask=cv2.dilate(mask,kern_dilate)     #Dilating
    #cv2.imshow('mask',mask)
    return mask

def find_blob(blob): #returns the red colored circle
    largest_contour=0
    cont_index=0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
            largest_contour=area
            cont_index=idx
                              
    r=(0,0,1,1)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
       
    return r,largest_contour

def target_hist(frame):
    hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    hist=cv2.calcHist([hsv_img],[0],None,[50],[0,255])
    return hist

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
      mask_red = segment_colour(frame)      #masking red the frame
      loct,area = find_blob(mask_red)
      x,y,w,h = loct
     
      #distance coming from front ultrasonic sensor
      distanceC = sonar(GPIO_TRIGGER2,GPIO_ECHO2)
      #distance coming from right ultrasonic sensor
      distanceR = sonar(GPIO_TRIGGER3,GPIO_ECHO3)
      #distance coming from left ultrasonic sensor
      distanceL = sonar(GPIO_TRIGGER1,GPIO_ECHO1)
             
      if (w*h) < 10:
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
            #if the ball is not found and the last time it sees ball in which direction, it will start to rotate in that direction
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
            if(area < 800): 
                  print("forward")
                  forward()
                  time.sleep(0.1)
                  stop()
                  time.sleep(0.00625)
            elif(area>=initial):
                  if(distanceC>10):
                        #it brings coordinates of ball to center of camera's imaginary axis.
                        if(centre_x<=-20 or centre_x>=20):
                              if(centre_x<0):
                                    flag=0
                                    print("right")
                                    turnright()
                                    time.sleep(0.05)
                              if(centre_x>0):
                                    flag=1
                                    print("left")
                                    turnleft()
                                    time.sleep(0.05)
                        
                        stop()
                        time.sleep(0.00625)
                  else:
                        stop()
                        time.sleep(0.01)
      '''
      elif(found == 1):
            print("distanceR = ", distanceR, ", distanceC = ", distanceC, ", distanceL = ", distanceL)
            print("area: ", area)
            if(distanceC > 20 and area < 600):
                  print("moving forward")
                  forward()
                  time.sleep(0.00625)
            if(centre_x <= -20 or centre_x >= 20):
                  if(centre_x<0):
                        print("turning right")
                        turnright()
                        time.sleep(0.00625)
                  elif(centre_x>0):
                        print("turning left")
                        turnleft()
                        time.sleep(0.00625)
      '''
      cv2.imshow("draw",frame)    
      rawCapture.truncate(0)  # clear the stream in preparation for the next frame
         
      if(cv2.waitKey(1) & 0xff == ord('q')):
            break

GPIO.cleanup() #free all the GPIO pins

```

# Bill of Materials

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Raspberry Pi 4.0 | microcontroller for the robot | $104.95 | <a href="https://www.pishop.us/product/raspberry-pi-4b-starter-kit-pro/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Raspberry Pi Camera | camera for object detection | $12.99 | <a href="https://a.co/d/2Kqccgh"> Link </a> |
|:--:|:--:|:--:|:--:|
| L298N H-Bridge | motor controller | $6.99 | <a href="https://a.co/d/jjrOBcA"> Link </a> |
|:--:|:--:|:--:|:--:|
| Ultrasonic Sensors (x3) | ultrasonic sensors for distance tracking | $8.99 | <a href="https://a.co/d/iPUL5EN"> Link </a> |
|:--:|:--:|:--:|:--:|
| Generic DC motors (x2) | motors and wheels for robot mobility | $13.59 | <a href="https://a.co/d/d8Io2eJ"> Link </a> |


<!--
# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)
To watch the BSE tutorial on how to create a portfolio, click here.
-->

# Starter Project
<iframe width="560" height="315" src="https://www.youtube.com/embed/Z2-qvxFV8pM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For my starter project, I created the Useless Box. This is a plain box with a single lever on the top of it. When this lever is flipped, an arm comes out of the box, flipping it back, and then going back into the box, reverting it to it's previous state. Additionally, an LED in the box turns green when the lever is flipped and the arm is moving towards it, red when the arm has hit the lever and is moving back into the box, and turns off when the arm is back in the box.

The major components of this project are a PCB, connecting a lever, an LED, a motor, and batteries. Additionally, there is a switch that is pressed by the arm when the box is closed, due the the unique shape of the arm. The LED is red when the lever is in its default state, green when the lever has been flipped by the user, but off when the switch is pressed. The batteries power the LED and motor, and the lever determines the state of the LED and the rotation direction of the motor. Surrounding the electronics is a black box, with a flush door on the top. The hoor has a hinge placed in a way so that the arm will push the door up when it flicks the lever and close the door after it moves back down.

When the lever is flicked by the user, the motor rotates, moving the arm towards the lever and away from the switch, causing the LED to turn green and the arm to rotate towards the lever. When the arms hits the lever, the LED changes to red and the motor starts rotating in the opposite direction, moving the arm back down until it reaches it default state. In this state, the switch is pressed, turning the LED off. 


