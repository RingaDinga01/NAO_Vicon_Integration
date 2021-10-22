#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

#Importing relevant modules
import sys

#Append the system path to the location of the NaoQi SDK for Nao modules
sys.path.append('/home/zachary/Downloads/pyNaoqi/lib/python2.7/site-packages')
import qi
import argparse
import almath
import math
import numpy as np
import time

#import the ros requirements
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

#Set ViconData position variables with global access
global vicondata
vicondata = None
global Vposx
Vposx = None
global Vposy
Vposy = None
global yaw_Nao
yaw_Nao = None 
global roll
roll = None


def main(session):
    """
    This example uses the getRobotPosition method.
    """
    # Get the services ALMotion & ALRobotPosture.
    useSensorValues = False
    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    initRobotPosition = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
    # Kill the movement that is already occuring
    #motion_service.killMove()

    #Initalize the arms for better walk stability   
    #leftArmEnable  = True
    #rightArmEnable = True
    #motion_service.setMoveArmsEnabled(leftArmEnable, rightArmEnable)

    # Wake up robot
    motion_service.wakeUp()

    # Send robot to Stand Init
    posture_service.goToPosture("StandInit", 0.5)

    # Initialize the move
    motion_service.moveInit()

    t_start = time.time()#Start the clock
    t_end = float('inf')#float('inf')##We want to do do this forever so set = inf
    t = 0 #set current time to 0 before loop begins
    freqRobot = 0.5 #Robot Step Frequency
    f =  0.018 #frequency
    r = 0.5 #radius of turn
    w = 2*math.pi*f
    Kp = 0.30 #Position Gain
    Kthy = 0.50 #Turn Rate Gain

    # Create some text files to write data
    NaoPos = open('NaoPosition.txt', 'w')
    PartPos = open('ParticlePosition.txt', 'w')
    TimeFile = open('Time.txt', 'w')
    SpeedFile = open('Speed.txt', 'w')
    TurnRateFile = open('TurnRate.txt', 'w')
    RollFile = open('Roll.txt','w')
    time.sleep(3)

    #Main Control Loop (CHANGE ROTATION OF COORDINATE SYSTEM BASED ON NAO LOCAL HEADING IN VICON)
    while(t<t_end):
        #Start Timer
        t=time.time()-t_start
        listener()
        if vicondata!= None:
            #Define Desired Global Position (x right y up)
            posx =2#+r*np.cos(w*t) #Current desired x global position (m)
            posy =0#r*np.sin(w*t)#r*np.sin(2*w*t)/2 #Current desired y globalposition (m)
            print "global theta", (math.atan2(posy,posx))*180/np.pi
            print "nao theta", (math.atan2(Vposy,Vposx))*180/np.pi

            #Transform these Global Coordinates (x right y up) into GlobalNAO (x up y left) by rotating around z axis clockwise relative to NAO global thy
            coordMatrixClock = np.matrix([[np.cos(math.pi/2), np.sin(math.pi/2)], [-np.sin(math.pi/2), np.cos(math.pi/2)]])
            PosGlobal= np.matrix([[posx],[posy]])
            PosNaoGlobal = np.matmul(coordMatrixClock,PosGlobal)

            #Writing Data to Files
            NaoPos.write("%s\n" % np.column_stack((Vposx,Vposy)))
            #Write particle pos to file
            PartPos.write("%s\n" % np.column_stack((posx,posy)))
            #Write Time Vector to file
            TimeFile.write("%s\n" % np.column_stack((t,t)))
            #Writing roll Data to File
            RollFile.write("%s\n" % np.column_stack((roll,Vposy)))

            #Setting Global Velocity 
            xGdesDeriv = 0#-r*w*np.sin(w*t) #Derivative of desired x global (m/s)
            yGdesDeriv = 0#r*w*np.cos(w*t)#+2*r*w*np.cos(2*w*t)/2 #Derivative of desired y global (m/s)

            #Get NAOs current Local heading from the Sim (Use VICON GLOBAL Heading with Vicon)
            cTheta = yaw_Nao#RobotPosition.theta #Grab NAOs Heading
            
            #Create matrix with Global difference in position (x right y up)
            PosErrorMat2 = np.matrix([[posx-Vposx],[posy-Vposy]])
	        print PosErrorMat2
            #Transform Global Difference (x right y up) to NAO Global (x up y left)
            PosErrorMatNao = np.matmul(coordMatrixClock,PosErrorMat2)
            #print PosErrorMatNao
            NAOx = PosErrorMatNao[0,0]
            NAOy = PosErrorMatNao[1,0]
            #Create the desired velocity vector in Global (x right y up)
            VdesGlobal = np.matrix([[xGdesDeriv],[yGdesDeriv]])

            #Create the desired velocity vector in Global NAO (x up y left)
            VdesGlobalNAO = np.matmul(coordMatrixClock,VdesGlobal)
            
            #Create Vc command Velocity in NAO Global (x up y left)
            Vcc = VdesGlobal+Kp*(PosErrorMat2)
            Vccx = Vcc[0,0]
            Vccy = Vcc[1,0]

            cTheta=(yaw_Nao)
            #Compute the Desired Global Thy
            ThyDes=(math.atan2(Vccy,Vccx))
            #print "Desired Angle", ThyDes*180/np.pi
            #Compute Desired turn angle error
            thyAng3 = ThyDes-cTheta
            #print "turn error" ,thyAng3*180/np.pi

            #Deal with the heading flip around the unit circle
            if thyAng3>np.pi:
                finalHeading = thyAng3-(2*np.pi)
            elif thyAng3<-np.pi:
                finalHeading = thyAng3+(2*np.pi)
            else:
                finalHeading = thyAng3
            print "final heading", finalHeading*180/np.pi   

            #Compute Desired turn rate
            turnRate = Kthy*finalHeading
            TurnRateFile.write("%s\n" % np.column_stack((turnRate,cTheta)))

            #Compute Desired Forward Speed
            vcxsquare = np.square(Vccx)
            vcysquare = np.square(Vccy)
            Sdes = math.sqrt(vcxsquare+vcysquare)

            #If sDes (Forward Speed) is greater than 0.06m/s then set it to 0.06m/s as this is what was found to be a stable speed for NAO in the real environment
            if Sdes>0.06:
                Sdes=0.06
            
            #Check if turn rate is greater than 0.53rad/s in both directions (NAO Max Turn Rate)
            if turnRate>0.5:
                turnRate=0.5
            if turnRate<-0.5:
                turnRate=-0.5
            
            #Write the the forward speed to a file 
            SpeedFile.write("%s\n" % np.column_stack((Sdes,t)))

            #Command the robot to move 
            motion_service.move(Sdes,0,turnRate, [["Frequency", freqRobot]])

            #print motion_service.getMoveConfig("Max")
            print "Global x", Vposx
            print "Global y",  Vposy
            print "Robot Yaw",  yaw_Nao, yaw_Nao*(180/np.pi)
            print "Speed", Sdes

            time.sleep(0.2)

       

#Callback method used to get the vicon data for NAO

def callback(data):
    #Set the x,y position of NAO from the vicon arena
    global Vposx
    global Vposy
    Vposx= data.transform.translation.x
    Vposy= data.transform.translation.y
    Vposz= data.transform.translation.z
    #rospy.loginfo('Pos x = %f', Vposx)
    #rospy.loginfo('Pos y = %f', Vposy)
    #rospy.loginfo('Pos z = %f', Vposz)

    Vquartx = data.transform.rotation.x
    Vquarty = data.transform.rotation.y
    Vquartz = data.transform.rotation.z
    Vquartw = data.transform.rotation.w

    euler_from_quaternion(Vquartx, Vquarty, Vquartz, Vquartw)
    global vicondata
    vicondata = data
def callback2(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global vicondata
    vicondata = data.data

# This function is used to convert the quartonion data into the roll,pitch and yaw in radians

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        global roll
        roll_x = math.atan2(t0, t1)
        roll = roll_x
        print "Roll = ", roll_x
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        global yaw_Nao
        yaw_Nao = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_Nao # in radians

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # Choose the topics that I want to subscribe to in ROS

    rospy.Subscriber('/vicon/Zac_NAO/Zac_NAO', TransformStamped, callback) ##Subscribing to NAO Position Data
    rospy.Subscriber('chatter', String, callback2)

if __name__ == "__main__":    
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.1.9",
                        help="Robot IP address. On robot or Local Naoqi: use '192.168.1.9'.")##127.0.0.1 or 192.168.1.22 or 192.168.1.9
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)

