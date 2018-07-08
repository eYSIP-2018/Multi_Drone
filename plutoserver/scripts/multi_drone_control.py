#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

# to inherit te Thread class and time for delay
import threading
import time

## To read input (no blocking)
import sys
import select
import math

from plutodrone.srv import *
from plutoserver.msg import floatar    #custom msg type defined to get the roll ,pitch and yaw of drone from another publisher
from plutodrone.msg import *
from plutoserver.msg import PlotData
from geometry_msgs.msg import  PoseArray
from whycon.srv import *
from pid_tune.msg import PidTune

#publisher instance for controlling the drone
#drone_0=rospy.Publisher('/drone_command_0', PlutoMsg, queue_size=1)
#drone_1=rospy.Publisher('/drone_command_1', PlutoMsg, queue_size=1)



# Number of nodes (drones to control)
NODES=1

# Vraibles to store the whycon positions of drones
pos_x=[0.0 for i in range(NODES)]
pos_y=[0.0 for i in range(NODES)]
pos_z=[0.0 for i in range(NODES)]

# variables to store the previous values of drones 
pos_px=[0.0,0.0,4.5]
pos_py=[0.0,0.0,4.5]
#pos_px=[-4.5,4.5,4.5]
#pos_py=[0.0,-4.5,4.5]
pos_pz=[0.0 for i in range(NODES)]

# variables to store the sensor values of drones
pitch=[0.0 for i in range(NODES)]
roll=[0.0 for i in range(NODES)]
alt=[0.0 for i in range(NODES)]
yaw=[0.0 for i in range(NODES)]
g_z=[0.0 for i in range(NODES)]

# fixed values 
fix_yaw=[0.0 for i in range(NODES)]
fix_x=[0.0,0.0,4.5]
fix_y=[0.0,0.0,4.5]
fix_alt=[18.0]*NODES
is_first_yaw=[True]*NODES

# flag varible to start the drone pid
exit_flag=False
start_flag=True
control_flag=True


# variables to store the PID values of drones
pid_pitch=[{'kp':12.0,'kd':610.0,'ki':4.0},{'kp':11.0,'kd':610.0,'ki':6.0},{'kp':18.0,'kd':610.0,'ki':6.0}]  # 8,240,15
pid_roll=[{'kp':11.0,'kd':610.0,'ki':4.0},{'kp':11.0,'kd':610.0,'ki':6.0},{'kp':18.0,'kd':610.0,'ki':6.0}]
pid_yaw=[{'kp':0.0,'kd':0.0,'ki':35.0},{'kp':10.0,'kd':3.0,'ki':0.0},{'kp':10.0,'kd':3.0,'ki':0.0}]
pid_throttle=[{'kp':55.0,'kd':77.0,'ki':2.0},{'kp':50.0,'kd':77.0,'ki':2.0},{'kp':40.0,'kd':77.0,'ki':2.0}]






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
     rospy.Subscriber('/Sensor_data_1',floatar,readData,1)
     rospy.Subscriber('/Sensor_data_2',floatar,readData,2)
     rospy.Subscriber('/pid_tuning',PidTune,change_PID)
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
      self.drone_debug=rospy.Publisher("drone_debug", PlotData, queue_size=1)
      #self.call()
      
   
   
   
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
   * Function Name: Natural
   * Input:   None
   * Output:  publish message to drone_command topic
   * Logic:   publish message to drone_command topic with default values
   *
   * Example Call: disarm()
   '''  
   def natural(self):
    cmd = PlutoMsg()
    cmd.pluto=self.node
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1500
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1500
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
    global pos_x,pos_y,pos_z,pitch,roll,g_z,yaw,alt,fix_yaw,fix_x,fix_y,fix_alt,start_flag,exit_flag,control_flag
    
    maxpropx=60.0
    maxpropy=60.0

    th_flag=False
    fix_xx=fix_x[self.node]
    fix_yy=fix_y[self.node]

    # storing sensor values from global list
    pitchs=pitch[self.node]
    rolls=roll[self.node]
    yaws=yaw[self.node]
    alts=alt[self.node]
    
    ## block to arm the drone

    self.disarm()
    self.disarm()
    self.disarm()
    print(self.node)
    self.arm()
    rospy.sleep(0.18)
    self.arm()
    rospy.sleep(0.18)
    self.disarm()
    self.disarm()
    self.natural()
    



    while start_flag:
      if exit_flag:
        sys.exit()  
    print("Node "+str(self.node)+" OK Started...")
    control_flag=False

    if True:
     self.disarm() 
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
    time_pre=time.time()
    
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


    plot=PlotData()

    
    diffz_filter=[0.0,0.0,0.0,0]
    diffy_filter=[0.0,0.0,0.0,0]
    diffx_filter=[0.0,0.0,0.0,0]
    z_filter_i=0
    y_filter_i=0
    x_filter_i=0


    rate=rospy.Rate(29)
    while(1):
       # delay according to drone processing rate
       #rospy.sleep(0.040)
       
       

       '''
       if th_flag:
        fix_xx=fix_x[self.node]
        fix_yy=fix_y[self.node]
       elif pos_z[self.node]<28:
        th_flag=True
       else:
        fix_xx=pos_x[self.node]
        fix_yy=pos_y[self.node] '''
           
       ######## PID to control the YAW of the drone  #################
       #erroryaw=fix_yaw[self.node]-yaw[self.node]
       gyro_z=g_z[self.node]
       if abs(gyro_z)<20:
         gyro_z=0

       erroryaw=g_z[self.node]
       '''if abs(erroryaw)>200:
         erroryaw=fyaw-(yaws+((abs(erroryaw)/erroryaw)*360))'''
       integral_yaw+=erroryaw/200 
       yaw_cmd=self.yaw['kp']*erroryaw+self.yaw['ki']*integral_yaw+self.yaw['kd']*(erroryaw-errorpreyaw)
       if yaw_cmd>200:
        yaw_cmd=200
       elif yaw_cmd<-200:
        yaw_cmd=-200
         
       #cmd.rcYaw=1500+yaw
       #print("yawww:",yaw);
       #cmd.rcYaw=1500
        
       ##########**************************############
       #         PID for pitch and roll of drone    ###
       ##########*************************#############
       #errorx=fix_x[self.node]-pos_x[self.node]
       #errory=fix_y[self.node]-pos_y[self.node]
       errorx=fix_x[self.node]-pos_x[self.node]
       errory=fix_y[self.node]-pos_y[self.node]
       diffx=prex-pos_x[self.node]
       diffy=prey-pos_y[self.node]
       if prex==0.0:
         diffx=0.0
         diffy=0.0
  
       
       # low pass filter to x
       diffx_filter[x_filter_i]=diffx
       diffx=diffx*0.4+diffx_filter[0]*0.3+diffx_filter[1]*0.3
       x_filter_i=(x_filter_i+1)%2
       
       
       # low pass filter to y
       diffy_filter[y_filter_i]=diffy
       diffy=diffy*0.4+diffy_filter[0]*0.3+diffy_filter[1]*0.3
       y_filter_i=(y_filter_i+1)%2
       
       
       
       # limit for difference if marker is not detected for a time
       if diffx>0.25:
        diffx=0.25
       elif diffx<-0.25:
        diffx=-0.25
       if diffy>0.25:
        diffy=0.25
       elif diffy<-0.25:
        diffy=-0.25
           
       #diffx=errorx-errorprex
       #diffy=errory-errorprey
       if abs(errorx)<2.8:
        integral_x+=(errorx/100)*self.pitch['ki']
       if abs(errory)<2.8: 
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
       
       LIMIT=200
       if pid_x>LIMIT:
        pid_x=LIMIT
       elif pid_x<-LIMIT:
        pid_x=-LIMIT
        
       if pid_y>LIMIT:
        pid_y=LIMIT
       elif pid_y<-LIMIT:
        pid_y=-LIMIT  
       
       
       

       ###*******************************************************************************************###
       ###                             PID for Height of Drone                                       ###
       ###*******************************************************************************************### 
       
       
       if alt[self.node]>600:
         alt[self.node]=prez
       
       ### calculating the error    
       #errorz=alt[self.node]-fix_alt[self.node]
       errorz=fix_alt[self.node]-pos_z[self.node]
       #diffz=errorz-errorprez   # alt[self.node]-prez
       #diffz=alt[self.node]-prez
       diffz=prez-pos_z[self.node]
       if prez==0.0:
         diffz=0.0

       if diffz<-1.5:
        diffz=-1.5
       elif diffz>1.5:
        diffz=1.5

       # low pass filter to z
       diffz=diffz*0.55+diffz_filter[0]*0.15+diffz_filter[1]*0.15+diffz_filter[2]*0.15
       diffz_filter[z_filter_i]=diffz
       z_filter_i=(z_filter_i+1)%3


       
       #print("diffz:",diffz)
       kdz=self.throttle['kd']
       kpz=self.throttle['kp']
       ### kdz is doubled when height is decreasing from fixes point  ###
       '''if pos_z[self.node]>fix_alt[self.node] and diffz<0:    # increase the kd if drone height is decreasing from set point
        kdz*=1.6
       elif pos_z[self.node]<fix_alt[self.node] and diffz>0: # reduce the kd if drone is crossing the set point height 
        kdz/=2
        kpz/=2 '''                                             # if drone is above the set point then reduce the kp to decrease the throttle gradualy 
        
       integral_z+=(errorz/200)*self.throttle['ki']
       #if integral_z<-35:
       # integral_z=-35
       
       
       # claculating proprotaional component
       propz=kpz*errorz
       if propz<-300:
        propz=-300
       elif propz>200:
        propz=200
       #print("propz:"+str(self.node)+":"+str(propz))  
       speedz=propz+integral_z+kdz*diffz
       #print("speedz:",speedz)

     
       
       
         
       ### To maintain the throttle upper value as 2000
       
       
       


       '''
       if speedz<0:
         if speedz<-250:
           speedz=-250
         temp=1-abs(speedz)/250.0
         pid_x=pid_x*temp
         pid_y=pid_y*temp
       
       if not(th_flag):
         pid_x=0.0
         pid_y=0.0'''

       throttle=1500-speedz 
       #speedx=1500-pid_x
       #speedy=1500-pid_y
       speedy=1500+pid_x
       speedx=1500-pid_y


       if throttle>2000:
        throttle=2000
       elif throttle<1000:
        throttle=1000 
       

       
       
       # assigning calculated values to PlutoMsg object
       #cmd.rcYaw=1500
       cmd.rcYaw=1500+yaw_cmd
       cmd.rcRoll=speedy
       cmd.rcPitch=speedx
       cmd.rcThrottle=throttle
       
       # debuging point
       #print("errorx:"+str(self.node)+": "+str(errorx))
       #print("errory:"+str(self.node)+": "+str(errory))
       #print("errorz:"+str(self.node)+": "+str(errorz))
       #print("error_yaw:"+str(self.node)+": "+str(erroryaw))
       #print("diffx:"+str(self.node)+": "+str(diffx))
       #print("diffy:"+str(self.node)+": "+str(diffy))
       
       #print("Throttle"+str(self.node)+": "+str(throttle))
       print("Yaw"+str(self.node)+": "+str(cmd.rcYaw))
       #print("Roll"+str(self.node)+": "+str(cmd.rcRoll))
       #print("Pitch"+str(self.node)+": "+str(cmd.rcPitch))
       
       
       # assigning values of current error to previous value variables
       errorpreyaw=erroryaw
       prex=pos_x[self.node]
       prey=pos_y[self.node]
       prez=pos_z[self.node]
       
       
       while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
        line = sys.stdin.readline()
        if line=="e\n":         #to stop the node with disarming the drone if e is pressed followed by enter
          cmd.rcThrottle=1300
          self.drone_control.publish(cmd)
          rospy.sleep(0.7)
          cmd.rcThrottle=1200
          self.drone_control.publish(cmd)
          rospy.sleep(0.7)
          cmd.rcThrottle=1000
          self.drone_control.publish(cmd)
          rospy.sleep(0.7)
          self.disarm()
          self.disarm()
          self.disarm()
          sys.exit() 
       else:
        # publishing to drone topic
        self.drone_control.publish(cmd)
        
        # publishing errors for debugging
        plot.error_x=errorx*20
        plot.error_y=errory*20
        plot.error_z=errorz*10
        plot.added_pid_x=-pid_x
        plot.added_pid_y=-pid_y
        plot.added_pid_z=-speedz
        self.drone_debug.publish(plot)
        rate.sleep()
        #rospy.sleep(0.033)

        #print("Time diff:"+str(self.node)+":"+str(time.time()-time_pre))
        #time_pre=time.time()





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
     tmp_x=[0.0 for i in range(NODES)]
     tmp_y=[0.0 for i in range(NODES)]
     tmp_z=[0.0 for i in range(NODES)]
     for i in range(length):
       tmp_x[i]=posear[i].position.x
       tmp_y[i]=posear[i].position.y
       tmp_z[i]=posear[i].position.z
     
     # temporary list to store the current postion of drones
     posp_x=[0 for i in range(NODES)]
     posp_y=[0 for i in range(NODES)]
     selected=[False for i in range(length)]
     selectedy=[False for i in range(length)]
     
     
     for x in range(length):
       small=100
       s_index=0
       for y in range(length):
          if small>abs(pos_px[y]-tmp_x[x])+abs(pos_py[y]-tmp_y[x]):
           if selected[y]==False:
             small=abs(pos_px[y]-tmp_x[x])+abs(pos_py[y]-tmp_y[x])
             s_index=y
             #print("small:"+str(small))
       pos_y[s_index]=tmp_y[x]
       #posp_y[sy_index]=x   
       pos_x[s_index]=tmp_x[x]
       pos_z[s_index]=tmp_z[x]
       #posp_x[sx_index]=x
       selected[s_index]=True
     
     
    
     
     # assigning current val to pre val array
     #print("pos_px:"+str(pos_x))
     #print("pos_py:"+str(pos_y))
     pos_px=pos_x
     pos_py=pos_y
     pos_pz=pos_z
     
     
     # debuging point 
     #print("pos_x:"+str(pos_x))
     #print("pos_y:"+str(pos_y))
     #print("tmp_x:"+str(tmp_x))
     #print("tmp_y:"+str(tmp_y))
     #print("ar_x:"+str(ar_x))
     #print("ar_y:"+str(ar_y))
     







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
    global pitch,roll,alt,yaw,fix_yaw,is_first_yaw,g_z
    pitch[x]=data.pitch
    roll[x]=data.roll
    yaw[x]=data.yaw
    alt[x]=data.alt
    g_z[x]=data.gz
    if is_first_yaw[x]:
      fix_yaw[x]=data.yaw+6
      is_first_yaw[x]=False
      print("initial values saved")  
    #debuging point 
    #print(yaw[x])
    #print(fix_yaw[x])




'''
* Function Name: change_PID
* Input:  PidTune data (custom msg defined to get the values from GUI)
* Output: save the pid values of pitch, roll, throttle in global variables.
* Logic:  the functions take PidTune object from publisher and save the corresponding vlaues to pid_pitch ,pid_roll and pid_throttle
*
* Example Call: change_PID(PidTune)
'''

def change_PID(data):
    global pid_pitch,pid_roll,pid_throttle
    x=data.Node
    pid_pitch[x]['kp']=data.Kp
    pid_pitch[x]['kd']=data.Kd
    pid_pitch[x]['ki']=data.Ki

    pid_roll[x]['kp']=data.Kp
    pid_roll[x]['kd']=data.Kd
    pid_roll[x]['ki']=data.Ki

    pid_throttle[x]['kp']=data.Kp_1
    pid_throttle[x]['kd']=data.Kd_1
    pid_throttle[x]['ki']=data.Ki_1
    
    print(pid_throttle)
    print(pid_pitch)
    print(pid_roll)


'''
* Function Name: draw_circle
* Input:  circle type and arc radius
* Output: Set target as points of circle arc
* Logic:  Itrate to update the target point such that the drone pass through the arc of a circle
*
* Example Call: draw_circle(num,arc)
'''


def draw_circle(num,arc):
   global fix_x,fix_y
   # drawing circle as one drone at center and other drawing the arc
   if num==0: 
    fix_x[0]=0.0
    fix_y[0]=0.0
    rad=3.14/180
    x=10
    for i in range(36):
          while  not(abs(pos_x[1]-fix_x[1])<0.4 and abs(pos_y[1]-fix_y[1]<0.4)):
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
             line = sys.stdin.readline()
             if line=="q\n":
              sys.exit()
          fix_x[1]=arc*math.sin(rad*(90-x*i))
          fix_y[1]=arc*math.cos(rad*(90-x*i))
          print("x:"+str(fix_x[1]))
          print("y:"+str(fix_y[1]))


   elif num==1:
     # drawing the circle which have both the drones on the arc of circle in exact opposite direction
    fix_x[0]=-fix_x[1]
    fix_y[0]=-fix_y[1]

    time.sleep(10)
    rad=3.14/180
    x=10
    for i in range(36):
          while  not(abs(pos_x[1]-fix_x[1])<0.5 and abs(pos_y[1]-fix_y[1])<0.5 and abs(pos_x[0]-fix_x[0])<0.5 and abs(pos_y[1]-fix_y[1])<0.5):
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
             line = sys.stdin.readline()
             if line=="q\n":
              sys.exit()
          fix_x[1]=arc*math.sin(rad*(90-x*i))
          fix_y[1]=arc*math.cos(rad*(90-x*i))
          fix_x[0]=-fix_x[1]
          fix_y[0]=-fix_y[1]
          print("x:"+str(fix_x[1]))
          print("y:"+str(fix_y[1]))      


'''
* Function Name: draw_square
* Input:  square type and arc radius
* Output: Set target as points of square nodes
* Logic:  Itrate to update the target point such that the drone pass through the nodes of a square
*
* Example Call: draw_square(num,size)
'''


def draw_square(num,size):
     global fix_x,fix_y


     if num==1:
        ar_x=[-size,-size,size,size]
        ar_y=[-size,size,size,-size]
        fix_x[2]=size
        fix_y[2]=size
        fix_x[0]=-fix_x[2]
        fix_y[0]=-fix_y[2]
        rad=3.14/180
        for i in range(5):
          while  not(abs(pos_x[2]-fix_x[2])<0.5 and abs(pos_y[2]-fix_y[2])<0.5 and abs(pos_x[0]-fix_x[0])<0.5 and abs(pos_y[0]-fix_y[0])<0.5):
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
             line = sys.stdin.readline()
             if line=="q\n":
              sys.exit()
          fix_x[0]=ar_x[i%4]
          fix_y[0]=ar_y[i%4]
          fix_x[2]=-fix_x[0]
          fix_y[2]=-fix_y[0]
          print("x:"+str(fix_x[0]))
          print("y:"+str(fix_y[0]))
          rospy.sleep(5)


'''
* Function Name: draw_triangle
* Input:  triangle size
* Output: Set target as points of triangle nodes
* Logic:  Itrate to update the target point such that the drone pass through the nodes of a triangle
*
* Example Call: draw_square(size)
'''

def draw_triangle(size):
     global fix_x,fix_y

     ar_x=[-size,-size/2,size,size+size/2,size,0.0]
     ar_y=[0.0,size/2,-size,0.0,size,size]
     fix_x[0]=ar_x[0]
     fix_y[0]=ar_y[0]

     fix_x[1]=ar_x[1]
     fix_y[1]=ar_y[1]

     fix_x[2]=ar_x[2]
     fix_y[2]=ar_y[2]
     
     for i in range(6):
          while  not(abs(pos_x[2]-fix_x[2])<0.5 and abs(pos_y[2]-fix_y[2])<0.5 and abs(pos_x[0]-fix_x[0])<0.5 and abs(pos_y[0]-fix_y[0])<0.5 and abs(pos_x[1]-fix_x[1])<0.5 and abs(pos_y[1]-fix_y[1])<0.5):
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
             line = sys.stdin.readline()
             if line=="q\n":
              sys.exit()
          fix_x[0]=ar_x[i%3]
          fix_y[0]=ar_y[i%3]
          fix_x[1]=ar_x[(i+1)%3]
          fix_y[1]=ar_y[(i+1)%3]
          fix_x[2]=ar_x[(i+2)%3]
          fix_y[2]=ar_y[(i+2)%3]
          print("x:"+str(fix_x[0]))
          print("y:"+str(fix_y[0]))
          if i%2==0:
           rospy.sleep(5)



if __name__ == '__main__':
    rospy.init_node('Multi_drone_control', anonymous=True)
    #print("working")


    # thread to read data from the drone sensor and whycon
    data_th=dataThread()
    data_th.start()

    #### Making instance of thread and starting thread to control the drones  #####
    drone0=controlThread(0,pid_pitch[0],pid_roll[0],pid_yaw[0],pid_throttle[0],'/drone_command_0')
    #drone1=controlThread(1,pid_pitch[1],pid_roll[1],pid_yaw[1],pid_throttle[1],'/drone_command_1')
    #drone2=controlThread(2,pid_pitch[2],pid_roll[2],pid_yaw[2],pid_throttle[2],'/drone_command_2')
    
    drone0.start()
    #drone1.start()
   # drone2.start()
    
    #draw_circle(0,5)
    while control_flag:
      while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
             line = sys.stdin.readline()
             if line=="p\n":
              start_flag=False
             elif line=="n\n":
              exit_flag=True
              rospy.sleep(1)
              sys.exit()

    rospy.sleep(6)
    #draw_triangle(3.5)
    #draw_square(1,4.5)

    rospy.spin()
   
    
    
    
    
    
