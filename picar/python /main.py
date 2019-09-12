

import RPi.GPIO as gpio
import time
import sys
import xbox


IN1 = 11
IN2 = 7
IN3 = 13
IN4 = 15
ENA = 19
ENB = 16

gpio.setmode(gpio.BCM)
gpio.setup(IN2, gpio.OUT)
gpio.setup(IN1, gpio.OUT)
gpio.setup(IN3, gpio.OUT)
gpio.setup(IN4, gpio.OUT)
gpio.setup(ENA ,gpio.OUT)
gpio.setup(ENB, gpio.OUT)
l=gpio.PWM(ENA,50)
r=gpio.PWM(ENB,50)
l.start(0)
r.start(0)
joy = xbox.Joystick()

def setSpeed(LSpeed,RSpeed):
    if (LSpeed > 0):
        gpio.output(IN2,True)
    	gpio.output(IN1,False)
        l.ChangeDutyCycle(LSpeed)
    elif (LSpeed<=0):
        gpio.output(IN2,False)
        gpio.output(IN1,True)
        l.ChangeDutyCycle(-LSpeed)
    if RSpeed > 0:
        gpio.output(IN3,True)
        gpio.output(IN4,False)
        r.ChangeDutyCycle(RSpeed)
    else:
        gpio.output(IN3,False)
        gpio.output(IN4,True)
        r.ChangeDutyCycle(-RSpeed)

    
while True:
    l_x_axis = joy.leftX()  #returns x axis of left joystick (-1.0 to 1.0)
    l_y_axis = joy.leftY()  #returns y axis of left joystick (-1.0 to 1.0)
    r_x_axis = joy.rightX() #returns x axis of right joystick (-1.0 to 1.0)
    r_y_axis = joy.rightY() #returns y axis of right joystick (-1.0 to 1.0)

    maxSpeed=15
    LSpeed = (joy.leftY()+joy.rightX())*maxSpeed
    RSpeed = (joy.leftY()-joy.rightX())*maxSpeed
    setSpeed(LSpeed,RSpeed)

    if joy.A():
        print 'A pressed'
    time.sleep(0.1)

#
#	setSpeed(10,10)
#	time.sleep(2)
#	setSpeed(-10,-10)
#	time.sleep(2)



def reverse(tf):
    gpio.output(7, False)
    gpio.output(11, True)
    gpio.output(13, False)
    gpio.output(15, True)
    time.sleep(tf)
    
def forward(tf):
    gpio.output(7, True)
    gpio.output(11, False)
    gpio.output(13, True)
    gpio.output(15, False)
    time.sleep(tf)
    
def turn_right(tf):
    gpio.output(7, True)
    gpio.output(11, False)
    gpio.output(13, False)
    gpio.output(15, True)
    time.sleep(tf)
    
def turn_left(tf):
    gpio.output(7, False)
    gpio.output(11, True)
    gpio.output(13, True)
    gpio.output(15, False)
    time.sleep(tf)

def stop(tf):
    gpio.output(7, False)
    gpio.output(11, False)
    gpio.output(13, False)
    gpio.output(15, False)
    time.sleep(tf)
    gpio.cleanup()

def key_input(event):
    init()
    print "Key:", event.char
    key_press = event.char
    sleep_time = 0.060
    
    if key_press.lower() == "w":
        forward(sleep_time)
    elif key_press.lower() == "s":
        reverse(sleep_time)
    elif key_press.lower() == "a":
        turn_left(sleep_time)
    elif key_press.lower() == "d":
        turn_right(sleep_time)
    elif key_press.lower() == "p":
        stop(sleep_time) 
    else:
        pass
