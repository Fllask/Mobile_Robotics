#!/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8
#the line above makes it more nicely executable on my mac, PATH may differ on different machines

from Thymio import Thymio
import os
import sys
import time
import numpy as np
import math as m
import pandas as pd
import math as m
import serial


# FUNCTION NEEDED TO MAKE IT GO STRAIGHT FROM ONE POINT TO AN OTHER ONE : 

# compute the speed in x and y direction knowing the speed of the robot: 
def speed_X_Y(Pos,NextGoal,SPEED):
    angle=m.atan2(NextGoal[1]-Pos[1],NextGoal[0]-Pos[0])
    #print("angle:",angle)
    speed=(SPEED*m.cos(angle),SPEED*m.sin(angle)) #speed in x and y 
    return speed

# state space equation of the robot when it is going straight 
def state_space_straight(Pos,speed,Ts):
   A=np.eye(2)  
   B=np.eye(2)*Ts
   PosNew=A.dot(np.transpose(Pos))+B.dot(np.transpose(speed)) #compute the new position of the robot at time t+ts
   return PosNew

#calculate the euler distance between two points 
def compute_distance (Pos,NextGoal):
    dist = m.sqrt((Pos[0]-NextGoal[0])**2 + (Pos[1]-NextGoal[1])**2 )
    return dist

# we choose the speed of the robot depending on our distance to the goal, we go slower when we are close to the goal to be more precise
def choose_speed(dist):
    if dist>20:
        SPEED=20 #SPEED is in cm/s
    elif 20>dist>10:
            SPEED=15
    elif 10>dist>5:
        SPEED=10
    else :
        SPEED=5
    #print("SPEED",SPEED)
    return SPEED

# Use the function define above to compute the prediction of the position at time t+ts only by giving it's initial position and next goal
def pos_predict(Pos,NextGoal,Ts):
    dist=compute_distance (Pos,NextGoal)
    SPEED=choose_speed(dist)
    speed=speed_X_Y(Pos,NextGoal,SPEED)
    #print("speed:",speed)
    PosPredict=state_space_straight(Pos,speed,Ts)
    #print("new pose",initPos)
    return PosPredict

# to check in which state we are (still need to go straight our if we have to turn
def estimate_straight_state(Pos,NextGoal):
    Dist=compute_distance(Pos,NextGoal)
    State=1
    #print(Dist)
    if Dist<0.5:
        State=0
    return State

def SPEED_conversion(SPEED): # conversion from SPEED in cm/s to vlue to put in motor.target
    MOTOR=SPEED*500/20 # for the moment, only a linear relation
    return MOTOR

#give instruction to the motor of the thymio : 
def thymio_straight(Pos,NextGoal):
    Dist=compute_distance (Pos,NextGoal)
    SPEED=choose_speed(Dist)
    MOTOR=SPEED_conversion(SPEED)
    MOTOR = MOTOR if MOTOR >= 0 else 2 ** 16 + MOTOR
    MOTOR=int(MOTOR)
        # Setting the motor speeds
    th.set_var("motor.left.target", int(MOTOR))
    th.set_var("motor.right.target", int(MOTOR))


# ALL THE FUNCTIONS NEEDED TO MAKE THE THYMIO TURN FROM ONE ANGLE TO THE OTHER :  

#calculate the angle between the x_axis and the array [pos1 pos2]
def calculate_angle(Pos1,Pos2):
    angle_value=m.atan2(Pos2[1]-Pos1[1],Pos2[0]-Pos1[0])
    return angle_value

#Use to calculate the difference between the angle of the robot and the angle that we want
def dist_angle(Angle,goalAngle):
    turn_angle=(goalAngle-Angle)*180/m.pi #turn angle is in degree 
    if turn_angle>180:
        turn_angle=turn_angle-360
    elif turn_angle<(-180):
        turn_angle=360+turn_angle
    return turn_angle

# choose a different speed rotation depending on our closeness to the goal angle 
def ROT_SPEED(turn_angle):
    if turn_angle>90:
        ROT=80
    elif 90>turn_angle>45:
        ROT=40
    elif 45>=turn_angle>5:
        ROT=20
    elif 5>=turn_angle>0:
        ROT=4
    elif -5<turn_angle<=0:
        ROT=-4
    elif -45<turn_angle<=-5:
        ROT=-20
    elif -90<turn_angle<=45:
        ROT=-40
    else:
        ROT=-80
    return ROT

# compute the new angle at time t+ts : 
def compute_angle(angle,ROT,Ts):
    NewAngle=angle+ROT/180*m.pi*Ts
    return NewAngle

#Use the functions above to compute the prediction of the angle at time t+ts by giving only the actual angle and the goal angle and the delta t
def theta_predict(Angle,goalAngle,Ts):
    turnAngle=dist_angle(Angle,goalAngle)
    W=ROT_SPEED(turnAngle)
    #print("W",W)
    AnglePredict=compute_angle(Angle,W,Ts)
    #print("angle",AnglePredict)
    return AnglePredict

#estimate if we are close enough to the goal angle and decide if we still have to turn or if we can start going straight
def estimate_angle_state(Angle,goalAngle):
    State=0
    turnAngle=dist_angle(Angle,goalAngle)
    #print("turnAngle",turnAngle)
    if abs(turnAngle)<0.5:
        State=1
    return State

# find the value that we have to give to the motor depending on the 
def angle_conversion(ROT): #convesrion of angular velocity to value to put in motor.target
    MOTOR=ROT*200/80 #pour l'instant juste une règle de 3 : 
    return MOTOR

#give an instruction to the motor to turn at a certain speed:  
def thymio_turn(Angle,goalAngle):
    turnAngle=dist_angle(Angle,goalAngle)
    ROT=ROT_SPEED(turnAngle)
    MOTOR=angle_conversion(ROT)
    if MOTOR>=0:
        MOTORL=MOTOR
        MOTORR=2**16-MOTOR
    else:
        MOTORL=2**16+MOTOR
        MOTORR=-MOTOR

    MOTORL=int(MOTORL)
    MOTORR=int(MOTORR)
    th.set_var("motor.left.target", MOTORL)
    th.set_var("motor.right.target", MOTORR)


#function to avoid an obstacle : 

def avoid_obstacle(thymio,Angle,InitialPos,NextPos,Ts): 
    state=3 
    while state==3:
        sensor=np.array(thymio["prox.horizontal"])
        right=sensor[4]+sensor[3] #sensors at the right
        left=sensor[0]+sensor[1] #sensors at the left 
        if right>left: #avoid by going left
            turn=0 #0 means turn left and 1 means turn right 
            ActualPos=avoid_obstacle_leftright(thymio,Angle,InitialPos,NextPos,Ts,turn)
        else:
            turn=1
            ActualPos=avoid_obstacle_leftright(thymio,Angle,InitialPos,NextPos,Ts,turn)
        return ActualPos
            
#to check if the obstacle is avoided and that we are back on the global path, we calculate the angle between the 
# x axis and the vector [Position_before_avoiding_obstacle next_goal_in_our_global_path] and the angle between the
# x axis and the vector [our_actual_position_during_obstacle_avoidance next_goal_in_our_global_path] 
# we calculate the difference between the two and if this difference is close to 0 that means that we are back again on our global path    
def test_obstacle_is_avoided(InitialPos,Pos,NextPos):
    angle1=calculate_angle(InitialPos,NextPos)
    angle2=calculate_angle(Pos,NextPos)
    diff=abs(angle1-angle2)
    print(diff)
    if diff<0.05:
        state=4
    else:
        state=1
    return state

#avoid the obstacle by going left our right depending on the position of the obstace
#if the obstacle is on the left : 
    # 0 : first the thymio turn left until it doesn't sense the obstacle 
    # 1 : after it moves straight at 4cm/s during 2 seconds (maybe have to change the speed) and check every 100ms if he is back to check if it is back to the global path
    #     if the thymio is back to the global path it it goes in state 4. Else, it goes in state 3
    # 2 : turn in the right direction until it sees the obstacle again. 
    # 3 : turn in the left direction until it doesn't sense the obstacle 
    # repeat 2-3-4 until the thymio avoid the obstacle 

    #4 : if the thymio has avoided the obstacle : turn until it reachs its original angle orientation (before local avoidance)

def avoid_obstacle_leftright(thymio,InitialAngle,InitialPos,NextPos,Ts,turn):
    state=0 #initial state (turn left or right depending on the value of turn)
    Angle=InitialAngle
    Pos=InitialPos
    A=np.eye(2) #use for state space equation
    B=np.eye(2)*Ts #use for state space equation
    MOTOR=100 #value to give to the motor
    SPEED=MOTOR*20/500 #speed in cm/s of the thymio
    const=0 # constant use to make sure that the thymio make one cycle before checking if the obstacle is avoided (else since at the begining the thymio is on the global path, thymio can estimate that the obstacle is avoided at the begining)
    if turn==0: #turn left
        idx_sensor=(3, 4) #need the measurements of sensor 3 and 4 (right sensors)
        ROT=MOTOR*80/200
    if turn==1: #turn right  
        idx_sensor=(1,0) #need the measurements of sensor 1 and 0 (left sensors)
        ROT=-MOTOR*80/200
    #print("actalpos",Pos)
    while 1:
        while state==0: #thymio is only one time in state 0 (at the begining to turn left or right)
            if turn==0:
                MOTORL=2**16-MOTOR
                MOTORR=MOTOR
            else:
                MOTORL=MOTOR
                MOTORR=2**16-MOTOR
            MOTORL=int(MOTORL)
            MOTORR=int(MOTORR)
            th.set_var("motor.left.target", MOTORL)
            th.set_var("motor.right.target", MOTORR)
            time.sleep(Ts)
            sensor=np.array(thymio["prox.horizontal"])
            #print("actualangle",Angle)
            Angle=compute_angle(Angle,-ROT,Ts) #compute actual angle 
            #print(sensor)
            if sensor[idx_sensor[1]]<1 and sensor[idx_sensor[0]]<1:
                state=1
                print("state1")
        if state==1: #go straight during 2 second 

            th.set_var("motor.left.target", MOTOR)
            th.set_var("motor.right.target", MOTOR)
            for i in range(0, 20): #loop to make the thymio turn during to seconds
                time.sleep(Ts)
                speed=(SPEED*m.cos(Angle),SPEED*m.sin(Angle))
                Pos=A.dot(np.transpose(Pos))+B.dot(np.transpose(speed)) #estimate the new position of the thymio (pos_x and pos_y)
                print("actualPos",Pos)
                if const==1:
                    state=test_obstacle_is_avoided(InitialPos,Pos,NextPos) #check if the obstacle is avoided
                if state==4:
                    print("state4")
                    while 1: #the thymio go back to it's initial angle 
                        Angle=theta_predict(Angle,InitialAngle,Ts) #estimate next angle at time t+ts
                        thymio_turn(Angle,InitialAngle) #make the thymio turn
                        time.sleep(Ts) #
                        finish=estimate_angle_state(Angle,InitialAngle) #check if we still need to turn or if we can go straight
                        print(finish)
                        print(Angle)
                        print(InitialAngle)
                        if finish==1:
                            return Pos
            state=2 #once the loop for is finished goes in state 2
                


        while state==2: # turn left or right to stay close to the obstacle 
            const=1
            if turn==1:
                MOTORL=2**16-MOTOR
                MOTORR=MOTOR
            else:
                MOTORL=MOTOR
                MOTORR=2**16-MOTOR
            MOTORL=int(MOTORL)
            MOTORR=int(MOTORR)
            th.set_var("motor.left.target", MOTORL)
            th.set_var("motor.right.target", MOTORR)
            time.sleep(Ts)
            sensor=np.array(thymio["prox.horizontal"])
            #print("actualangle",Angle)
            Angle=compute_angle(Angle,ROT,Ts)
            #print("index")
            #print(idx_sensor[1])

            if sensor[idx_sensor[1]]>1000: #once we are close enough to the obstacle goes in state right 
                
                state=3
                print("state3")

        while state==3: #turn right or left so that we are sure to avoid the obstacle before going straight again 
            if turn==0:
                MOTORL=2**16-MOTOR
                MOTORR=MOTOR
            else:
                MOTORL=MOTOR
                MOTORR=2**16-MOTOR
            MOTORL=int(MOTORL)
            MOTORR=int(MOTORR)
            th.set_var("motor.left.target", MOTORL)
            th.set_var("motor.right.target", MOTORR)
            time.sleep(Ts)
            sensor=np.array(thymio["prox.horizontal"])
            #print("actualangle",Angle)
            Angle=compute_angle(Angle,-ROT,Ts) #compute actual angle 
            if sensor[idx_sensor[1]]<1:
                state=1
                print("state3to1")




# function to make the thymio work: will decide the state of the thymio (turn, go straight or local avoidance)
def Tymio_run(thymio,a,Ts):
    j=0
    next=0 #will be use to estimate where we are (our global path is composed of several points), next is use to check between which point our thymio is 
    state=0 #means state=TURN
    Angle=0 #radian
    NextAngle=calculate_angle(a[0],a[1]) #we want our thymio to be oriented with this angle
    Pos=a[0]
    while True: # continue until it reach the goal 
        j=j+1  #timer
        if state==0:  #turn
            Angle=theta_predict(Angle,NextAngle,Ts) #estimate next angle at time t+ts
            thymio_turn(Angle,NextAngle) #make the thymio turn
            time.sleep(Ts) #
            state=estimate_angle_state(Angle,NextAngle) #check if we still need to turn or if we can go straight
            if state==1:
                next+=1  
                NextPosition=a[next]
                print("going straight")
               
        if state==1: #go straigth
            Pos=pos_predict(Pos,NextPosition,Ts)
            thymio_straight(Pos,NextPosition)
            time.sleep(Ts)
            state=estimate_straight_state(Pos,NextPosition)
            if state==0 and next<4: #len(a)
                NextAngle=calculate_angle(a[next],a[next+1])
                print("turning")
            elif state==0 and next==4:   #len(a)
                print ("arrive at destination")
                th.set_var("motor.left.target", 0)
                th.set_var("motor.right.target", 0)
                
                return 0 
        
            
            x= np.array(thymio["prox.horizontal"])
            if sum(x[0:4])>2000: # threshold a modifié 
                #enter in avoid local obstacle mode 
                Pos=avoid_obstacle(thymio,Angle,Pos,NextPosition,Ts)
            else:
                print("straight")






a=[(0,0),(50.0,0),(50,50),(0,50),(0,0)] #points for the direction
Ts=0.1 #sampling time


th = Thymio.serial(port="\\.\COM3", refreshing_rate=0.1)
time.sleep(3) # To make sure the Thymio has had time to connect

Tymio_run(th,a,Ts) #test if it works 