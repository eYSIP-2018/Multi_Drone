#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

# to inherit te Thread class and time for delay
import threading
import time

## To read input (no blocking)
import sys
import select

from plutodrone.srv import *
from plutoserver.msg import floatar    #custom msg type defined to get the roll ,pitch and yaw of drone from another publisher
from plutodrone.msg import *
from geometry_msgs.msg import  PoseArray
from whycon.srv import *

#publisher instance for controlling the drone
drone_0=rospy.Publisher('/drone_command_0', PlutoMsg, queue_size=1)
drone_1=rospy.Publisher('/drone_command_1', PlutoMsg, queue_size=1)



# Number of nodes (drones to control)
NODES=2

# Vraibles to store the whycon positions of drones
pos_x=[0.0]*NODES
pos_y=[0.0]*NODES
pos_y=[0.0]*NODES

# variables to store the previous values of drones 
pos_px=[0.0]*NODES
pos_py=[0.0]*NODES
pos_pz=[0.0]*NODES













cmd = PlutoMsg()
cmd.rcRoll = 1500
cmd.rcPitch = 1500
cmd.rcYaw =1500
cmd.rcThrottle =1500
cmd.rcAUX1 =1500
cmd.rcAUX2 =1500
cmd.rcAUX3 =1500
cmd.rcAUX4 =1500


######   Thread To get the whycon poses and pitch,roll, magnetometer data of drone   ##########
class controlThread(threading.Thread):
   
   
   
   def __init__(self,num):
      threading.Thread.__init__(self)
      self.node = num
   
   
   
   '''
   * Function Name: run
   * Input: none
   * Output: none
   * Logic:  calling listen() and access_data() functions when data is available at publisher . This function 
   *         continues run parallel to main function call because it is in different thread 
   * Example Call: When instance's start() function is called i.e. threadinstance.start()
   '''

   def run(self):
    disarm(self.node)
    disarm(self.node)
    disarm(self.node)
    print("working")
    print(self.node)
    arm(self.node)
    rospy.sleep(0.18)
    arm(self.node)
    rospy.sleep(0.18) 
     









'''
* Function Name: listen
* Input: PoseArray 
* Output: save the positions of the drone and runner in the global variables and update the drone_timediff according to the time gap after
          this function is called 
* Logic:  The poses of both the markers is taken and based on the z value of the markers the position of drone and runner is catched
          The runner marker is quit smaller than the drone marker .  
*
* Example Call: listen(PoseArray)
'''       
            
def image_data(data):
     global pos_x,pos_y,pos_z,pos_px,pos_py,pos_pz,NODES
     
     # temporary list to store the current postion of drones
     tmp_x=pos_x
     tmp_y=pos_y
     tmp_z=pos_z
     
     # to find the x position of drones according to pre position
     for x in tmp_x:
       small=100
       s_index=0
       for y in pos_px:
          if small>abs(pos_px[y]-tmp_x[x]):
             small=abs(pos_px[y]-tmp_x[x])
             s_index=y
          pos_x[s_index]=tmp_x[x]
          
                    
     # to find the y position of drones according to pre position
     for x in tmp_y:
       small=100
       s_index=0
       for y in pos_py:
          if small>abs(pos_py[y]-tmp_y[x]):
             small=abs(pos_py[y]-tmp_y[x])
             s_index=y
          pos_y[s_index]=tmp_y[x]   
          
                                        
     # to find the x position of drones according to pre position
     for x in tmp_z:
       small=100
       s_index=0
       for y in pos_pz:
          if small>abs(pos_pz[y]-tmp_z[x]):
             small=abs(pos_pz[y]-tmp_z[x])
             s_index=y
          pos_z[s_index]=tmp_z[x]    
     
     
     
     
     
          
     seq=data.header.seq     
     posear=data.poses
     zpos=0.0
     
     ##### if no of markers drone_detected is two #####
     if len(posear)==2:
      markers=2
      runner_detected=1
      runner_detect_counter+=1
      if caveflag==0:
       cavecount=0
      elif runner_detect_counter>6:
       cavecount=0 
      tmpx=posear[0].position.x
      tmpy=posear[0].position.y
      tmpz=posear[0].position.z
      
      ####*****************************************************************************#####
      #### Whycon detect marker's z value based on the radius of the marker drone_detected   #####
      #### when the marker position is not at the center of the camera, then the marker#####
      #### become the eclipse hence the z calculated by the whycon decreases as marker #####
      #### postion is in either side of camera , by a factor z . So we multiply the z  #####
      #### value with a factor a(0.3) hence this will give approximate the equal       #####
      #### of Z at every position if the height of the marker is same                  #####
      ####*****************************************************************************#####
      
      if abs(tmpx)>abs(tmpy):
         tmpz+=abs(tmpx)*0.2
      else:
         tmpz+=abs(tmpy)*0.2
      
      tmpfx=posear[1].position.x
      tmpfy=posear[1].position.y
      tmpfz=posear[1].position.z
      
      if abs(tmpfx)>abs(tmpfy):
         tmpfz+=abs(tmpfx)*0.2
      else:
         tmpfz+=abs(tmpfy)*0.2
      
      #### if the z value of marker is less then 31 then the marker is of drone    
      if tmpz<22:
        drone_detected=1
        x=tmpx
        y=tmpy
        z=tmpz
        fpx=tmpfx
        fpy=tmpfy
        zpos=tmpfz
      elif tmpfz<22:
        drone_detected=1
        x=tmpfx
        y=tmpfy
        z=tmpfz
        fpx=tmpx
        fpy=tmpy
        zpos=tmpz
     else:
       tmpx=posear[0].position.x
       tmpy=posear[0].position.y
       tmpz=posear[0].position.z
      
       if abs(tmpx)>abs(tmpy):
         tmpz+=abs(tmpx)*0.2
       else:
         tmpz+=abs(tmpy)*0.2 
       markers=1 
       #### if the z value of marker is less then 31 then the marker is of drone 
       if tmpz<22:
        drone_detected=1
        #cavecount+=1
        runner_detect_counter=0
        x=tmpx
        y=tmpy
        z=tmpz
       else:
        runner_detected=1
        runner_detect_counter+=1
        if caveflag==0:
         cavecount=0
        elif runner_detect_counter>6:
         cavecount=0 
        fpx=tmpx
        fpy=tmpy
        zpos=tmpz
        
        
    
    
    
     ##### Updating drone_timediff vairable according to the time difference between the last drone_detected image #####
     if drone_detected==1:
       timenow=time.time()
       if (timenow-drone_timepre)>drone_timediff:      #### updating the drone_timediff if current drone_timediff is greater than the maximum time difference 
                                           #### in a time 
         drone_timediff=timenow-drone_timepre   
       
       drone_timepre=timenow     
    
     ##### Updating runner_timediff vairable according to the time difference between the last drone_detected image #####
     if runner_detected==1:
       timenow=time.time()
       if (timenow-runner_timepre)>runner_timediff:      #### updating the timediff if current timediff is greater than the maximum time difference 
                                           #### in a time 
         runner_timediff=timenow-runner_timepre   
       
       runner_timepre=timenow   


















'''
* Function Name: arm
* Input: None
* Output: publish message to drone_command topic
* Logic:  publish message to drone_command topic for arm the drone with required delay
*
* Example Call: arm()
'''

def arm(node):
    global drone_0,drone_1
    cmd = PlutoMsg()
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1000
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1500
    if node==0:
     drone_0.publish(cmd)
    elif node==1:
     drone_1.publish(cmd) 
    rospy.sleep(0.18)

'''
* Function Name: disarm
* Input:   None
* Output:  publish message to drone_command topic
* Logic:   publish message to drone_command topic for disarming the drone with required delay
*
* Example Call: disarm()
'''
    
def disarm(node):
    global drone_0,drone_1
    cmd = PlutoMsg()
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1300
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1200
    if node==0:
     drone_0.publish(cmd)
    elif node==1:
     drone_1.publish(cmd) 
    rospy.sleep(1)  




if __name__ == '__main__':
    rospy.init_node('Multi_drone_control', anonymous=True)
    #print("working")
    #### Making instance of thread and starting thread to receive the whycon and drone sensor values   #####
    drone0=controlThread(0)
    drone1=controlThread(1)
    drone0.start()
    drone1.start()
    rospy.spin()
   
    
    
    
    
    
