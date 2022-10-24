#!/usr/bin/env python

#export DISPLAY=:0.0

from cmath import pi
from time import sleep
import turtle
import math
import rospy
import datetime
from sensor_msgs.msg import Range
import os
import RPi.GPIO as GPIO

BuzzerPin = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(BuzzerPin, GPIO.OUT) 
GPIO.setwarnings(False)

b = GPIO.PWM(BuzzerPin, 500)
b.start(90)
sleep(3)
b.stop()

os.environ['DISPLAY']=':0.0'

Sonar_0 = Sonar_1 = Sonar_2 = Sonar_3 = Sonar_4 = 0

def callback_sonar_0(data):
    global Sonar_0
    Sonar_0 = data.range


def callback_sonar_1(data):
    global Sonar_1
    Sonar_1 = data.range


def callback_sonar_2(data):
    global Sonar_2
    Sonar_2 = data.range


def callback_sonar_3(data):
    global Sonar_3
    Sonar_3 = data.range


def callback_sonar_4(data):
    global Sonar_4
    Sonar_4 = data.range


if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/pi_sonar/sonar_0', Range, callback_sonar_0)
    rospy.Subscriber('/pi_sonar/sonar_1', Range, callback_sonar_1)
    rospy.Subscriber('/pi_sonar/sonar_2', Range, callback_sonar_2)
    rospy.Subscriber('/pi_sonar/sonar_3', Range, callback_sonar_3)
    rospy.Subscriber('/pi_sonar/sonar_4', Range, callback_sonar_4)

    window = turtle.Screen()
    window.setup(width=1.0, height=1.0, startx=None, starty=None)
    turtle.tracer(False)

    turtle.setworldcoordinates(0, 0, 100, 100)
    turtle.hideturtle()

    #sonars from rigt to left
    s0 = turtle.Turtle()
    s1 = turtle.Turtle()
    s2 = turtle.Turtle()
    s3 = turtle.Turtle()
    s4 = turtle.Turtle()

    s0.penup()
    s1.penup()
    s2.penup()
    s3.penup()
    s4.penup()

    s0_checked = False
    s0_last = Sonar_0
    s1_checked = False
    s1_last = Sonar_1
    s2_checked = False
    s2_last = Sonar_2
    s3_checked = False
    s3_last = Sonar_3
    s4_checked = False
    s4_last = Sonar_4


    while not rospy.is_shutdown():
        s0.clear()
        s1.clear()
        s2.clear()
        s3.clear()
        s4.clear()

        s4.goto(10, 0)
        s4.shape("square")
        s4.shapesize(0.1+Sonar_4*20, 5)
        #s4.write("sonar 4: {:.2f} m".format(Sonar_4), font=('Arial', '14', 'normal'))
        if (abs(s4_last - Sonar_4) > 0.1 or s4_checked):
            s4_checked = True
            s4.color('green')

        s3.goto(30, 0)
        s3.shape("square")
        s3.shapesize(0.1+Sonar_3*20, 5)
        #s3.write("sonar 3: {:.2f} m".format(Sonar_3), font=('Arial', '14', 'normal'))
        if (abs(s3_last - Sonar_3) > 0.1 or s3_checked):
            s3_checked = True
            s3.color('green')

        s2.goto(50, 0)
        s2.shape("square")
        s2.shapesize(0.1+Sonar_2*20, 5)
        #s2.write("sonar 2: {:.2f} m".format(Sonar_2), font=('Arial', '14', 'normal'))
        if (abs(s2_last - Sonar_2) > 0.1 or s2_checked):
            s2_checked = True
            s2.color('green')

        s1.goto(70, 0)
        s1.shape("square")
        s1.shapesize(0.1+Sonar_1*20, 5)
        #s1.write("sonar 1: {:.2f} m".format(Sonar_1), font=('Arial', '14', 'normal'))
        if (abs(s1_last - Sonar_1) > 0.1 or s1_checked):
            s1_checked = True
            s1.color('green')

        s0.goto(90, 0)
        s0.shape("square")
        s0.shapesize(0.1+Sonar_0*20, 5)
        #s0.write("sonar 0: {:.2f} m".format(Sonar_0), font=('Arial', '14', 'normal'))
        if (abs(s0_last - Sonar_0) > 0.1 or s0_checked):
            s0_checked = True
            s0.color('green')

        turtle.update()

        if (s0_checked and s1_checked and s2_checked and s3_checked and s4_checked):
            for i in range(2):
                b.start(90)
                sleep(1)
                b.stop()
                sleep(1)
            exit()
        
        rospy.sleep(0.1)
