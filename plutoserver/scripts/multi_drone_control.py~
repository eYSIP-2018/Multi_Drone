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
#drone_0=rospy.Publisher('/drone_command_0', PlutoMsg, queue_size=1)
#drone_1=rospy.Publisher('/drone_command_1', PlutoMsg, queue_size=1)



# Number of nodes (drones to control)
NODES=2

# Vraibles to store the whycon positions of drones
pos_x=[0.0]*NODES
pos_y=[0.0]*NODES
pos_z=[0.0]*NODES

# variables to store the previous values of drones 
pos_px=[-4.0,4.0]
pos_py=[0.0]*NODES
pos_pz=[0.0]*NODES

# variables to store the sensor values of drones
pitch=[0.0]*NODES
roll=[0.0]*NODES
alt=[0.0]*NODES
yaw=[0.0]*NODES

# fixed values 
fix_yaw=[0.0]*NODES
fix_x=[-4.0,4.0]
fix_y=[0.0,0.0]
fix_alt=[70.0]*NODES
is_first_yaw=[True]*NODES


# variables to store the PID values of drones
pid_pitch=[{'kp':25.0,'kd':700.0,'ki':10.0}]*NODES   # 8,240,15
pid_roll=[{'kp':25.0,'kd':700.0,'ki':10.0}]*NODES
pid_yaw=[{'kp':50.0,'kd':3.0,'ki':0.0}]*NODES
pid_throttle=[{'kp':11.0,'kd':80.0,'ki':1.5}]*NODES




cmd = PlutoMsg()
cmd.rcRoll = 1500
cmd.rcPitch = 1500
cmd.rcYaw =1500
cmd.rcThrottle =1500
cmd.rcAUX1 =1500
cmd.rcAUX2 =1500
cmd.rcAUX3 =1500
cmd.rcAUX4 =1500






######   Thread To get the whycon poses and pitch,roll, magnetometer data from drone   ##########
class dataThread(threading.Thread):
   
   '''
   * Function Name: run
   * Input: none
   * Output: none
   * Logic:  calling listen() and access_data() functions when data is available at publisher . This function 
   *         continues run parallel to main function call because it is in different thread 
   * Example Call: When instance's start() function is called i.e. threadinstance.start()
   '''

   def run(self):
     rospy.Subscriber('/whycon/poses',PoseArray,image_data)
     rospy.Subscriber('/Sensor_data_0',floatar,readData,0)
     #rospy.Subscriber('/Sensor_data_1',floatar,readData,1)
     rospy.spin() 
     
     








######   Thread To control drone   ##########
class controlThread(threading.Thread):
   # constructor to define class variables
   def __init__(self,num,pitch,roll,yaw,throttle,topic):
      threading.Thread.__init__(self)
      self.node = num
      self.pitch=pitch
      self.roll=roll
      self.yaw=yaw
      self.throttle=throttle
      self.drone_control=rospy.Publisher(topic, PlutoMsg, queue_size=1)
   
   
   
   '''
   * Function Name: arm
   * Input: None
   * Output: publish message to drone_command topic
   *  Logic:  publish message to drone_command topic for arm the drone with required delay
   *
   * Example Call: arm()
   '''
   def arm(self):
    cmd = PlutoMsg()
    cmd.pluto=self.node
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1000
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1500
    self.drone_control.publish(cmd) 
    rospy.sleep(0.18)
   
    
   '''
   * Function Name: disarm
   * Input:   None
   * Output:  publish message to drone_command topic
   * Logic:   publish message to drone_command topic for disarming the drone with required delay
   *
   * Example Call: disarm()
   '''  
   def disarm(self):
    cmd = PlutoMsg()
    cmd.pluto=self.node
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1300
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1200
    self.drone_control.publish(cmd) 
    rospy.sleep(1) 
   
   
    
   '''
   * Function Name: run
   * Input: none
   * Output: none
   * Logic:  calling listen() and access_data() functions when data is available at publisher . This function 
   *         continues run parallel to main function call because it is in different thread 
   * Example Call: When instance's start() function is called i.e. threadinstance.start()
   '''

   def run(self):
    global pos_x,pos_y,pos_z,pitch,roll,yaw,alt,fix_yaw,fix_x,fix_y,fix_alt
    
    maxpropx=70.0
    maxpropy=70.0
    
    # storing sensor values from global list
    pitchs=pitch[self.node]
    rolls=roll[self.node]
    yaws=yaw[self.node]
    alts=alt[self.node]
    
    ## block to arm the drone
    self.disarm()
    self.disarm()
    self.disarm()
    print("working")
    print(self.node)
    self.arm()
    rospy.sleep(0.18)
    self.arm()
    rospy.sleep(0.18)
    
    prex=0.0
    prey=0.0
    prez=0.0
    errorpreyaw=0.0
    integral_yaw=0.0
    integral_x=0.0
    integral_y=0.0
    integral_z=0.0
    
    
    # PlutoMsg obj to publish data to pluto
    cmd = PlutoMsg()
    cmd.pluto=self.node
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1000
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1500
    
    diffz_filter=[0.0,0.0,0.0,0]
    diffy_filter=[0.0,0.0,0.0,0]
    diffx_filter=[0.0,0.0,0.0,0]
    z_filter_i=0
    y_filter_i=0
    x_filter_i=0
    
    while(1):
       # delay according to drone processing rate
       rospy.sleep(0.025)
           
       ######## PID to control the YAW of the drone  #################
       erroryaw=fix_yaw[self.node]-yaw[self.node]
       '''if abs(erroryaw)>200:
         erroryaw=fyaw-(yaws+((abs(erroryaw)/erroryaw)*360))'''
       integral_yaw+=erroryaw/200 
       yaw_cmd=self.yaw['kp']*erroryaw+self.yaw['ki']*integral_yaw+self.yaw['kd']*(erroryaw-errorpreyaw)
       if yaw_cmd>500:
        yaw_cmd=500
       elif yaw_cmd<-500:
        yaw_cmd=-500
         
       #cmd.rcYaw=1500+yaw
       #print("yawww:",yaw);
       #cmd.rcYaw=1500
        
       ##########**************************############
       #         PID for pitch and roll of drone    ###
       ##########*************************#############
       errorx=fix_x[self.node]-pos_x[self.node]
       errory=fix_y[self.node]-pos_y[self.node]
       diffx=prex-pos_x[self.node]
       diffy=prey-pos_y[self.node]
       
       # low pass filter to x
       diffx=diffx*0.45+diffx_filter[0]*0.20+diffx_filter[1]*0.20+diffx_filter[2]*0.20
       diffx_filter[x_filter_i]=diffx
       x_filter_i=(x_filter_i+1)%3
       
       
       # low pass filter to y
       diffy=diffy*0.45+diffy_filter[0]*0.20+diffy_filter[1]*0.20+diffy_filter[2]*0.20
       diffy_filter[y_filter_i]=diffy
       y_filter_i=(y_filter_i+1)%3
       
       
       
       # limit for difference if marker is not detected for a time
       if diffx>0.2:
        diffx=0.2
       elif diffx<-0.2:
        diffx=-0.2
       if diffy>0.2:
        diffy=0.2
       elif diffy<-0.2:
        diffy=-0.2
           
       #diffx=errorx-errorprex
       #diffy=errory-errorprey
       if abs(errorx)<2.2:
        integral_x+=(errorx/100)*self.pitch['ki']
       if abs(errory)<2.2: 
        integral_y+=(errory/100)*self.pitch['ki']
       
       
       prox=self.pitch['kp']*errorx
       proy=self.roll['kp']*errory
       
       
       #### To set the upper and lower bound on the proportional value so that drone move at consistant speed if target is far away ###
       factx=1
       facty=1
       rerrorx=1.0
       rerrory=1.0
       if errorx!=0:
        factx=errorx/abs(errorx)                         # to check the sign of error
        if errory!=0:
         rerrorx=abs(errorx/errory)                      # ratio of errors of x and y so drone reach taget at same time in both axis
         
       if errory!=0: 
        facty=errory/abs(errory)
        if errorx!=0:
         rerrory=abs(errory/errorx)
       if abs(prox)>maxpropx:
         prox=maxpropx*factx
         if rerrorx<1.0:
           prox*=rerrorx
       if abs(proy)>maxpropy:
         proy=maxpropy*facty
         if rerrory<1.0:
           proy*=rerrory 
       #print("prox:",prox,"proy:",proy)          
       pid_x=prox+integral_x+self.pitch['kd']*(diffx)
       pid_y=proy+integral_y+self.roll['kd']*(diffy)
       
       LIMIT=100
       if pid_x>LIMIT:
        pid_x=LIMIT
       elif pid_x<-LIMIT:
        pid_x=-LIMIT
        
       if pid_y>LIMIT:
        pid_y=LIMIT
       elif pid_y<-LIMIT:
        pid_y=-LIMIT  
       
       speedx=1500-pid_x
       speedy=1500-pid_y
       

       ###*******************************************************************************************###
       ###                             PID for Height of Drone                                       ###
       ###*******************************************************************************************### 
       
       
       if alt[self.node]>600:
         alt[self.node]=prez
       
       ### calculating the error    
       errorz=alt[self.node]-fix_alt[self.node]
       #diffz=errorz-errorprez   # alt[self.node]-prez
       diffz=alt[self.node]-prez
       # low pass filter to z
       diffz=diffz*0.55+diffz_filter[0]*0.15+diffz_filter[1]*0.15+diffz_filter[2]*0.15
       diffz_filter[z_filter_i]=diffz
       z_filter_i=(z_filter_i+1)%3
       
       print("diffz:",diffz)
       kdz=self.throttle['kd']
       kpz=self.throttle['kp']
       ### kdz is doubled when height is decreasing from fixes point  ###
       if alt[self.node]<fix_alt[self.node] and diffz<0:    # increase the kd if drone height is decreasing from set point
        kdz*=1.8
       elif alt[self.node]>fix_alt[self.node] and diffz>0: # reduce the kd if drone is crossing the set point height 
        kdz/=2
        kpz/=2                                              # if drone is above the set point then reduce the kp to decrease the throttle gradualy 
        
       integral_z+=(errorz/200)*self.throttle['ki']
       if integral_z<-35:
        integral_z=-35
       #if alts<falt and errorz>3 and diffz<2:
       # kpz*=1
       
       # claculating proprotaional component
       propz=kpz*errorz
       if propz<-300:
        propz=-300
       elif propz>200:
        propz=200
       print("propz:",propz)  
       speedz=propz+integral_z+kdz*diffz
       #print("speedz:",speedz)
       ### To maintain the throttle upper value as 2000
       throttle=1500-speedz
       if throttle>2000:
        throttle=2000
       elif throttle<1000:
        throttle=1000 
       
       
       
       
       # assigning calculated values to PlutoMsg object
       cmd.rcYaw=1400
       cmd.rcYaw=1500+yaw_cmd
       cmd.rcRoll=speedy
       cmd.rcPitch=speedx
       cmd.rcThrottle=throttle
       
       # debuging point
       print("errorx:"+str(errorx))
       print("errory:"+str(errory))
       print("errorz:"+str(errorz))
       print("error_yaw:"+str(erroryaw))
       print("diffx:",str(diffx))
       print("diffy:",str(diffy))
       
       print("Throttle:"+str(throttle))
       print("Yaw:"+str(cmd.rcYaw))
       print("Roll:"+str(cmd.rcRoll))
       print("Pitch:"+str(cmd.rcPitch))
       
       
       # assigning values of current error to previous value variables
       errorpreyaw=erroryaw
       prex=pos_x[self.node]
       prey=pos_y[self.node]
       prez=alt[self.node]
       
       
       while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
        line = sys.stdin.readline()
        if line=="e\n":         #to stop the node with disarming the drone if e is pressed followed by enter
          self.disarm()
          self.disarm()
          self.disarm()
          sys.exit() 
       else:
        # publishing to drone topic
        self.drone_control.publish(cmd)





'''
* Function Name: image_data
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
     
     posear=data.poses
     length=len(posear)
     tmp_x=[0.0]*NODES
     tmp_y=[0.0]*NODES
     tmp_z=[0.0]*NODES
     for i in range(length):
       tmp_x[i]=posear[i].position.x
       tmp_y[i]=posear[i].position.y
       tmp_z[i]=posear[i].position.z
     
     # temporary list to store the current postion of drones
     
     
     selected=[False]*length
     # to find the x position of drones according to pre position
     for x in range(length):
       smallx=100
       smally=100
       s_index=0
       for y in range(length):
          if smallx>abs(pos_px[y]-tmp_x[x]):
           if selected[y]==False:
             small=abs(pos_px[y]-tmp_x[x])
             s_index=y
       pos_x[s_index]=tmp_x[x]
       selected[s_index]=True   
     
     selected=[False]*length               
     # to find the y position of drones according to pre position
     for x in range(length):
       small=100
       s_index=0
       for y in range(length):
          if small>abs(pos_py[y]-tmp_y[x]):
           if selected[y]==False:
             small=abs(pos_py[y]-tmp_y[x])
             s_index=y
       pos_y[s_index]=tmp_y[x]
       selected[s_index]=True     
           
                                        
     # to find the x position of drones according to pre position
     for x in range(length):
       small=100
       s_index=0
       for y in range(length):
          if small>abs(pos_pz[y]-tmp_z[x]):
             small=abs(pos_pz[y]-tmp_z[x])
             s_index=y
       pos_z[s_index]=tmp_z[x]
          
     
     
     # assigning current val to pre val array
     print("pos_px:"+str(pos_px))
     print("pos_py:"+str(pos_py))
     pos_px=pos_x
     pos_py=pos_y
     pos_pz=pos_z
     
     # debuging point 
     print("pos_x:"+str(pos_x))
     print("pos_y:"+str(pos_y))
     print(pos_z)
     print("tmp_x:"+str(tmp_x))
     print("tmp_y:"+str(tmp_y))
     
     '''     
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












'''
* Function Name: readData
* Input:  floatar data (custom msg defined to get the values from drone)
* Output: save the pitch, roll, yaw in global variables   . Call the derive function according to the target and chase value 
* Logic:  the functions take floatar object from publisher and save the corresponding vlaues to pitchs ,rolls and yaws
          the first yaw value is stored in fyaw as fixed yaw reference  
          if chase value is 0 then chase follow the target according to the waypoint position , if the chase value is 1 then 
          target values are updating according to the runner position 
*
* Example Call: readData(floatar,int)
'''

def readData(data,x):
    global pitch,roll,alt,yaw,fix_yaw,is_first_yaw
    pitch[x]=data.pitch
    roll[x]=data.roll
    yaw[x]=data.yaw
    alt[x]=data.alt
    if is_first_yaw[x]:
      fix_yaw[x]=data.yaw-4
      is_first_yaw[x]=False
      
    #debuging point 
    #print(yaw[x])
    #print(fix_yaw[x])




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
    #### Making instance of thread and starting thread to control the drones  #####
    drone0=controlThread(0,pid_pitch[0],pid_roll[0],pid_yaw[0],pid_throttle[0],'/drone_command_0')
    #drone1=controlThread(1,pid_pitch[1],pid_roll[1],pid_yaw[1],pid_throttle[1],'/drone_command_1')
    
    #drone0.start()
    #drone1.start()
    
    # thread to read data from the drone sensor and whycon
    data_th=dataThread()
    data_th.start()

    rospy.spin()
   
    
    
    
    
    
