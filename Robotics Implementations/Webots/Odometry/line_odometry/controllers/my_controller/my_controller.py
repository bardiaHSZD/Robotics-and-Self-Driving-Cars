from controller import Robot
import numpy as np

import math

MAX= 6.24

robot = Robot()

timestep = int(robot.getBasicTimeStep())

gs = []
speedl, speedr = 0, 0
positon = [0, 0, 0]
xw,yw,dw,w,dx = 0,0,0,0,0
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

for i in range(3):
    gs.append(robot.getDevice("gs" + str(i)))
    gs[-1].enable(timestep)
  
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)



while robot.step(timestep) != -1:
    read = []
   
    for gsensor in gs:
        read.append(gsensor.getValue())
    
    if  read[1]< 400:
        speedl = MAX*0.5
        speedr = MAX*0.5
    elif read[2] < 550:
        speedr = MAX*-0.05
        speedl = MAX*0.3 
    elif read[0] < 550:
        speedr = MAX*0.3
        speedl = MAX*-0.05
    
    right_motor.setVelocity(speedr) 
    left_motor.setVelocity(speedl)
    
    
    dx = (32 / (2000))*((speedl * 0.0201 + speedr * 0.0201)) 
    xw = (np.cos(math.radians(w)) * dx)+xw
    
    dw = (32 *180/ (52*3.1415))*((-speedl * 0.0201 + speedr * 0.0201))
    w = w + dw
    yw = yw + np.sin(math.radians(w)) * dx
    
    
    
    Ed = (yw**2 + xw**2 )**(1/2)
    print("Elucidan distance =", Ed,"x =", xw, " y =", yw, " w =", w)
    pass
