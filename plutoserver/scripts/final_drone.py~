#!/usr/bin/env python


'''
* Team Id : eYRC#489

* Author List : Sunil Kumar, Dhruv , Krishan Kumar, Avseq 

*

* Filename:  final_drone.py

* Theme:  Chaser Drone  

* Functions:
 derive(float,float,float,float,float,float,float,float,float)
 listen(PoseArray), arm(), disarm(), access_data(floatar), talker(), main(), run(), set_targets(targets)

* Classes :   controlThread(Thread), targetThread(Thread)

* Global Variables: kp, kd, ki, fyaw, pitchs, rolls, yaws, altpre, rollpre, pitchpre, errorprex, errorprey, errorprez
  errorpreyaw, integralyaw, integralx, integraly, integralz, integralp, integralr, fpx, fpy, fpz, cmd, prefpx, prefpy, falt
  drone_timepre, drone_timediff, chase, stableflag, seq, preseq, landed, markers, errorprex, errorprey, drone_detected, diffprex, diffprey 
  althist, zhist, ahindex, ahindex, coughtflag, landflag, cavecount, caveflag, precaveflag, stablehovering

*

'''



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
move=rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

# constants for Image co-ordinates
#kpx=14
#kpy=15 
kpx=13
kpy=17    
kdx=230
kdy=230   #for 350mah battery
'''
kpx=18
kpy=19
kdx=200 
kdy=200    #for 600mah battery'''

ki=4.0

kdz=11
# Fixed yaw of drone during flight (updated by access_data at startup)
fyaw=0.0

# Pitch , roll and yaw of drone 
pitchs=0.0
rolls=0.0
yaws=0.0
alts=0.0
altpre=0.0

rollpre=0.0
pitchpre=0.0


x=0.0
y=0.0
z=0.0

prex=0.0
prey=0.0
errorprez=0.0

errorpreyaw=0.0
integralyaw=0.0

integralx=0.0
integraly=0.0
integralz=0.0

integralp=0.0
integralr=0.0

#Fixed (HOLD) positions for x, y and z position
fpx=0.0
fpy=0.0
prefpx=0.0
prefpy=0.0
fpz=29.0
falt=33
flag=0
tmpflag=0
hoverflag=0

#maxpropx=36             #### Upper bound value for the proportional conribution
#maxpropy=34
maxpropx=35
maxpropy=35


i=0
wpx=[-0.5,5.0,6.0,-0.5]
wpy=[0.4,-4.0,5.3,0.4]

difx=[0.0,0.0,0.0]
dify=[0.0,0.0,0.0]

drone_timepre=0.04      #### To get the time differece in marker detection if markers are not detected regularly
drone_timediff=0.04

runner_timepre=0.04
runner_timediff=0.04

runner_hold_time=0      #### To check if runner is on a node i.e. stopped for some time and saving that position
runner_haltx=0.0
runner_halty=0.0
runner_detected=0

runner_detect_counter=0

cave_fpx=0.0
cave_fpy=0.0

chase=0
stableflag=0

seq=0
preseq=0

landed=0
markers=0
errorprex=0.0
errorprey=0.0

drone_detected=0                       #to check whether marker drone_detected during a fixed time period
diffprex=0.0
diffprey=0.0                     #to save the change in position of x and y i.e. which can be used if marker not drone_detected

althist=[0.0,0.0,0.0,0.0,0.0]    #to store the history of altitude of drone from laser 
ahindex=0                        #althistory index pointer
zhist=[0.0,0.0,0.0,0.0,0.0]      #to store the history of altitude from whycon detection
zhindex=0                        #zhistory index pointer
coughtflag=0                     #to check the runner is cought or not
landflag=0                       #to check the stablity of the drone when it is on runner

cavecount=0                      # to count the time for which runner is not drone_detected
caveflag=0                       # flag to check that runner is under the cave or not
precaveflag=0                    # flag to save the previous value of the caveflag

stablehovering=0                  # flag to land the chased with a stable value if runner is under the cave


yawflag=0


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
   
   '''
   * Function Name: run
   * Input: none
   * Output: none
   * Logic:  calling listen() and access_data() functions when data is available at publisher . This function 
   *         continues run parallel to main function call because it is in different thread 
   * Example Call: When instance's start() function is called i.e. threadinstance.start()
   '''

   def run(self):
     rospy.Subscriber('/whycon/poses',PoseArray,listen)
     rospy.Subscriber('/magdata',floatar,access_data)
     rospy.spin() 
     
     
     
     
 ######   Thread To change the targets at runtime    ##########    
class targetThread(threading.Thread):
   
   '''
   * Function Name: run
   * Input: none
   * Output: none
   * Logic:  calling set_targets function of this thread to change the no of targets at runtime depending on a give condition 
   * Example Call: When instance's start() function is called i.e. threadinstance.start()
   '''

   def run(self):
     global preseq,seq,targets
     self.set_targets(2)
     while 1:
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
            line = sys.stdin.readline()
            if line=="o\n":         #to stop the thread
             sys.exit()
             
        #####Check if the no of poses received now and previous values are same then no target is detecting     #######
        ##### Hence change the target to 1 and check after every 600 milli seconds if two markers are available #######
        ##### If present markers are two then targets remain 2                                                  #######
        rospy.sleep(0.15)   
        if seq==preseq:                
           set_targets(1)
           rospy.sleep(0.6)
           set_targets(2) 
        preseq=seq  
      
     
   '''
   * Function Name: set_targets
   * Input: no of targets as agruements
   * Output: none 
   * Logic:  changiing the no of targets using whycon reset service 
   * Example Call: set_targets(self,2)
   ''' 
    
   def set_targets(self,x):
     rospy.wait_for_service('/whycon/reset')
     try:
        targets=rospy.ServiceProxy('/whycon/reset',SetNumberOfTargets)
        obj=targets(x)
     except rospy.ServiceException, e:
        print "Service call failed: %s"%e  
     

     
'''
* Function Name: derive
* Input:  whycon marker position i.e. float x, y and z 
          PID constants for image co-ordinates i.e. float kpx, kpy, kdx, kcy, , ki and kd
          fixed(HOLD) x , y and z positions
* Output: the desired roll, pitch and throttle values for drone calculated with PID 
* Logic:  The function take different args and values from global values and calculate the desired values for yaw, roll
          pitch, throttle . The PID calculated from the image co-ordinates is added with the Self valancing PID values of the 
          drone . The self balanced is maintained by taking the drone pitch, roll, yaw values
*
* Example Call: derive(float,float,float,float,float,float,float,float)
'''
       
def derive(x,y,z,kpx,kpy,kin,kdx,kdy,fpxx,fpyy,fpzz,detect):
       global prex,prey,fpz,ki,maxpropx,maxpropy,runner_timediff,cave_fpx,fpx,fpy,cave_fpy,runner_detect_counter,runner_hold_time,runner_haltx,runner_halty,runner_detected,drone_detected,althist,stablehovering,zhist,zhindex,coughtflag,ahindex,cavecount,caveflag,precaveflag,landflag,prefpx,prefpy,hoverflag,diffprex,diffprey,tmpflag,i,landed,markers,errorprex,errorprey,chase,altpre,stableflag,drone_timepre,drone_timediff,difx,dify,flag,cmd,integralp,integralr,errorprez,integralx,integralz,integraly,integralyaw,errorpreyaw,fyaw,falt,pitchs,rolls,alts,yaws,pitchpre,rollpre
       
       
       ######## PID to control the YAW of the drone  #################
       erroryaw=fyaw-yaws
       '''if abs(erroryaw)>200:
         erroryaw=fyaw-(yaws+((abs(erroryaw)/erroryaw)*360))'''
       integralyaw+=erroryaw/200 
       yaw=15.8*erroryaw+1.5*integralyaw+6*(erroryaw-errorpreyaw)
       cmd.rcYaw=1500+yaw
       #print("yawww:",yaw);
       #cmd.rcYaw=1500
        
        
       diff_fpx=prefpx-fpx
       diff_fpy=prefpy-fpy
       
       #print("diff_fpxx:",diff_fpx,"diff_diff_fpy:",diff_fpy)
       if abs(diff_fpx)>0.4:
        diff_fpx=0.4*(abs(diff_fpx)/diff_fpx)
        
       if abs(diff_fpy)>0.4:
        diff_fpy=0.4*(abs(diff_fpy)/diff_fpy)
        
       #print("diff_fpx:",diff_fpx,"diff_fpy:",diff_fpy)
         
       diff_fpx=(diff_fpx*0.04)/runner_timediff
       diff_fpy=(diff_fpy*0.04)/runner_timediff
       runner_timediff=0.04
       
       if abs(diff_fpx)<0.3 and abs(diff_fpy)<0.3:
         runner_hold_time+=1
       else:
         runner_hold_time=0
         
       if runner_hold_time>5 and runner_detected==1:
         runner_haltx=fpx
         runner_halty=fpy    
        
       ##########**************************############
       #         PID from Image co-ordinates        ###
       ##########*************************#############
       errorx=fpxx-x
       errory=fpyy-y
       diffx=prex-x
       diffy=prey-y
       #diffx=errorx-errorprex
       #diffy=errory-errorprey
       if abs(errorx)<2.2:
        integralx+=errorx/100
       if abs(errory)<2.2: 
        integraly+=errory/100
         
       
       ##### To check the stablity of the drone on a fixed target position ######    
       if abs(errorx)<0.6 and abs(errory)<0.6 and stableflag==1:
         flag+=1
       else:
         flag=0 
        
       
       ###### Moving the drone with a consistant speed if drone is far away from the target  #######
         
       '''if abs(errorx)>=1.6:
        kpx/=1.3
        kdx*=0.7 
       if abs(errory)>=1.6:
        kpy/=1.3
        kdy*=0.7'''
        
       
       ##### If the marker is at corner positions then whycon is unable to detect the exact position of the marker ####
       ##### Hence the position of marker changes with +-1 even if marker is stabe , So such unstable noise values ####
       ##### are very vulnerable for derivative term of PID . Hence to avoid this a LOW PASS filter is applied to  ####
       ##### differece in values of linear data to remove the noise                                                ####
       
       if abs(x)>8: 
        difx[0]=difx[1]
        difx[1]=difx[2]
        difx[2]=diffx 
        diffx=difx[0]*0.3+difx[1]*0.3+difx[2]*0.4
      
       if abs(y)>6: 
        dify[0]=dify[1]
        dify[1]=dify[2]
        dify[2]=diffy 
        diffy=dify[0]*0.3+dify[1]*0.3+dify[2]*0.4
       
       
       ###### To bind the change in position values to upper and lower limit ######
       
       if diffx>0.34:
          diffx=0.34
       elif diffx<-0.34:
          diffx=-0.34  
       if diffy>0.34:
          diffy=0.34
       elif diffy<-0.34:
          diffy=-0.34 
       
       
       
       ##### When the drone marker is not drone_detected for a moment then the difference between the current and previous posoion #####
       ##### become high which is vulnerable for derivative term of the PID . Hence the normal referesh time is 0.04 and if  #####
       ##### if image drone_detected after a time more than 0.04 then the differece is devided by the time to get the real speed   ##### 
       
       diffx=(diffx*0.04)/(drone_timediff)
       diffy=(diffy*0.04)/(drone_timediff)
       drone_timediff=0.04
           
          
       ##### If the marker is not drone_detected then the current position of the drone is considered as the last detect position #####
       ##### but if drone was moving then the difference in speed is considred as previous so that Derivative term of PID   #####
       ##### can controll the drone motion even if drone marker is not detecting . This prevent the drone form going out    #####
       ##### from arena in some situations and making drone stable if marker is not drone_detected in some situations             #####
       
       if drone_detected==0:
        diffx=diffprex
        diffy=diffprey
       
       if chase==1:
        if hoverflag==1:
         kpx=29
         kpy=28
         kdx*=1.2
         kdy*=1.2
        else:
         kpx=22
         kpy=20
         kdx*=0.9
         kdy*=0.9
       
       prox=kpx*errorx
       proy=kpy*errory
       
       #### To set the upper and lower bound on the proportional value so that drone move at consistant speed if target is far away ###
       factx=1
       facty=1
       rerrorx=1.0
       rerrory=1.0
       if errorx!=0:
        factx=errorx/abs(errorx)
        if errory!=0:
         rerrorx=abs(errorx/errory)
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
       speedx=prox+kin*integralx+kdx*(diffx)
       speedy=proy+kin*integraly+kdy*(diffy)
       
       ### To bound the maximum error ouput as +/- 69  ###
       if speedx>69:
        speedx=69
       elif speedx<-69:
        speedx=-69
       if speedy>69:
        speedy=69
       elif speedy<-69:
        speedy=-69
       
       
       ####*********************************************************************************************####
       ### The error calculated from Image cordinats is added with self balancing feedback loop values  #####
       ### The pitch is trimmed with -2 and rolls trimmed with 3 i.e. mid pitch val is 2 and mid roll  #####
       ### val is -3   The output is bounded with max 1600 and min 1400                                #####
       ### ********************************************************************************************#####
          
       valx=0.0
       valy=0.0
     
       ####assigning the variables to x and y direction  
       
       
       #print("valx:",speedx,"valy:",speedy)    
       valx=1500-speedx
       valy=1500-speedy
       #print("controlled")
       
       if valx>1600:
        valx=1600
       if valx<1400:
        valx=1400
       if valy>1600:
        valy=1600
       if valy<1400:
        valy=1400 
       
       ### Assigning the calculated values to global object variables  ###
       cmd.rcPitch=valx
       cmd.rcRoll=valy
       
       
       if caveflag==1 and abs(cave_fpx-fpxx)<5 and abs(cave_fpy-fpyy)<5 and fpxx!=0 and fpyy!=0 and runner_detected==1 and runner_detect_counter>6:
        cavecount=0
                  
       ###*******************************************************************************************###
       ###                             PID for Height of Drone                                       ###
       ###                                                                                           ###
       ### If the drone is going upward it can be stopped by decreasing the throttle by very less    ###
       ### values but if drone height is decreasing then it need more throttle to oppose the decrease###
       ### in height as well as to achieve the desired height . Hence when drone attain height more  ###
       ### than the desired point then it should be decreased with very less amount i.e. half and if ###
       ### height is decreasing fron the desired point then throttle should be increased rapidly to  ###
       ### to maintain the desired height .                                                          ###
       ### Hence  if height                                                                          ###
       ### decreasing then kd is doubled so that there will be rapid increase in throttle value      ###
       ### according to rate of decrease in height of the drone                                      ###
       ###*******************************************************************************************### 
       
       
       
       
          
       
       ### To calulate the movement of the drone changing the target error according the the movementum of drone  ###
       ### i.e.  if drone is moving towards the target with a speed x then the target postion is shifted towards  ###
       ### the drone so that it start landing before the target came under the drone hence smooth landing even if ###
       ### the drone is coming with some speed                                                                    ###
       
       #print("haltx:",runner_haltx,"halty:",runner_halty)
       #print("fpx:",fpxx,"fpy:",fpyy)
       #print("errorx:",errorx,"errory:",errory)
       errorxx=errorx
       erroryy=errory
       if landed==0:
        if diffx!=0 and errorx!=0 and chase==1:
                if abs(errorx)<2 and (abs(diffx)/diffx)!=(abs(errorx)/errorx):    #decreasing the error if drone is going towards the target
                 errorxx*=1-(abs(diffx)*2)
                else:                                                             #increasing the error id drone is going away from the target
                 errorxx*=1+(abs(diffx)*2)  
       
        if diffy!=0 and errorx!=0 and chase==1:
                if abs(errory)<2 and (abs(diffy)/diffy)!=(abs(errory)/errory):
                 errory*=1-(abs(diffy)*2)
                else:
                 erroryy*=1+(abs(diffy)*2)   
       
       #print("errorxx:",errorxx,"erroryy:",erroryy)
        
       if alts>300:
         alts=0 
       
       ######To check if runner is under the cave #######
       #print("cavecount:",cavecount,"caveflag:",caveflag)
       if cavecount>11:
          caveflag=1
       elif caveflag==1:
          caveflag=0   
       
      
       
       
       
       ### Sorting the list to get the smallest and largest values of altitude of previous 5 samples
       templistz=sorted(zhist)
       zhist[zhindex]=z
       zhindex+=1
       zhindex%=5
       
       templist=sorted(althist)
       althist[ahindex]=alts
       ahindex+=1
       ahindex%=5
       #print(templist)
       #### if the change is greater than 4 in previous five samples
       if templist[4]-alts>4 and abs(errorx)<1.1 and abs(errory)<1.1 and chase==1 and z-templistz[0]<3.4 and caveflag==0 and hoverflag !=1:
         hoverflag=1
         falt=7
       elif (templist[0]-alts<-6 or abs(errorx)>1.6 or abs(errory)>1.6 or cavecount>15) and hoverflag !=0 and chase==1:
         hoverflag=0 
        
       #print("hoverflag:",hoverflag)
       #print("falt:",falt)
       
       
       ###if drone is hovering then no need to achive the target altitude according to laser sensor  
       '''if hoverflag==1:
         errorz+=10'''  
       
       
       ## decreasing the altitude of chaser if it is hovering over the runner  
       if hoverflag==1 and falt>3  and chase==1:
         falt-=1
       elif chase==1 and abs(errorx)<2 and abs(errory)<2 and falt>13:
         falt-=1  
       elif chase==1 and hoverflag==0:
         falt=23 
            
       ### tmpflag is used to lock the increment of i if drone landed in arena instead of runner  ####
       #if flag>2 and chase==0 and stableflag==1 and tmpflag==0 and stablehovering==0:
       if flag>2 and chase==0 and stableflag==1 and stablehovering==0:
         i+=1
         ### To reset the overall error after a waypoint access  ###
         integralx=0.0
         integraly=0.0
         flag=0
         stableflag=0
       
       ###To takeoff again if drone not landed on runner###
       if stablehovering==1 and alts>12 and caveflag==0:
         i+=1
         stablehovering=0
         #tmpflag=0
         
       '''if flag>2 and falt>15 and chase==1:
          falt-=1
          flag=0'''
        
       '''if abs(errorx)<0.6 and abs(errory)<0.6 and falt<10:
         falt=0'''   
       #print("falt:",falt)   
       #errorz=falt-alts
       
       
       
       ### if drone is gone up than desired height then decrease the throttle value as half of error  ###
       '''if errorz>1:
        errorz/=2
        errorprez/=2
        kiz=1.5'''
       ### calculating the error    
       errorz=alts-falt
       
       #kdz=30
       #kpz=15
       kdz=17
       kpz=8
       diffz=errorz-errorprez   # alts-altspre
       #print("diffz:",diffz)
       
       ### kdz is doubled when height is decreasing from fixes point  ###
       '''if alts>falt and diffz<0 and hoverflag==0:
        #kdz=40
        kdz=15
       if hoverflag==1:
        kpz=25 '''
       '''if alts>falt: 
        kpz=15'''
       if landed!=1: 
        integralz+=errorz/100
       #if alts<falt and errorz>3 and diffz<2:
       # kpz*=1
       speedz=kpz*errorz+2*integralz+kdz*diffz
       #print("speedz:",speedz)
       ### To maintain the throttle upper value as 2000
       th=1500-speedz
       
       #print("Height",th)
       
           
       
       ### landing condition of the chaser 
       
       if (hoverflag==1 and abs(errorx)<0.4 and abs(errory)<0.4 and chase==1 and alts<8) or (hoverflag==1 and abs(errorxx)<0.7 and abs(erroryy)<0.7 and (abs(diffx)>0.15 or abs(diffy)>0.15) and alts<8):
          #print("drop landing") 
          th=1000
          landed=1
     
       elif abs(errorx)<0.4 and abs(errory)<0.4 and alts<6 and chase==1:
         th=1000
         landed=1
       
       if hoverflag==1 and abs(errorx)<0.6 and abs(errory)<0.6 and chase==1 and alts<12:
          landflag+=1
          if falt>0:
           falt-=2
       else:
          landflag=0 
          
       if landflag>5:
          #print("stable landing")
          th=1000
          landed=1 
       
       
          
        #### checking the landed position of the chaser
           
       if landed==1 and th>1000:
          #if markers==1 and abs(errorprex-errorx)<0.5 and abs(errorprey-errory)<0.5 :
          if markers<2:
            #landflag+=1
            th=1000
          elif markers==2 and abs(errorx)<1.8 and abs(errory)<1.8:
            th=1000
          elif markers==2: 
            tmpflag=1
            stablehovering=1
            #print("re-takeoff for wrong land4444")
            i=3
            wpx[i]=x
            wpy[i]=y
            chase=0
            landed=0 
            hoverflag=0
            falt=24 
       
       if landed==1:
          if abs(prex-x)<0.2 and abs(errorprex-errorx)<0.2 and abs(prey-y)<0.2 and abs(errorprey-errory)<0.2:
             coughtflag+=1
          else:
             coughtflag=0
          
          if coughtflag>12:      
             print("Runner Caught!!!!!")
             #print("errorx:",errorx,"errory:",errory)
             
       
             
       if caveflag==1 and chase==1 and stablehovering==0:
         chase=0
         stablehovering=1
         #print("Waiting for runner to came out from cave")
         i=3
         if abs(fpxx-runner_haltx)>abs(fpyy-runner_halty) and fpxx-runner_haltx!=0:
          wpy[i]=fpy
          wpx[i]=fpxx+4*(abs(fpxx-runner_haltx)/(fpxx-runner_haltx))
         
         elif fpy-runner_halty!=0:
          wpx[i]=fpxx
          wpy[i]=fpyy+4*(abs(fpyy-runner_halty)/(fpyy-runner_halty))
         
         if abs(wpx[i])>5:
           wpx[i]=5*(abs(wpx[i])/wpx[i])
         if abs(wpy[i])>5:
           wpy[i]=5*(abs(wpy[i])/wpy[i])          
         cave_fpx=fpxx
         cave_fpy=fpyy
         #wpx[i]=x
         #wpx[j]=y 
         '''if falt>5:
           falt-=1
         if alts<7:
           th=1000'''
       '''elif precaveflag==1 and caveflag==0 and chase==0 and stablehovering==1:
            tmpflag=1
            stablehovering=0
            #print("re-takeoff for wrong land")
            i=3
            wpx[i]=x
            wpy[i]=y
            chase=0
            landed=0 
            hoverflag=0
            falt=20'''
        
       '''if stablehovering==1 and caveflag==1 and abs(errorx)<1.0 and abs(errory)<1.0:
          if falt>5:
            falt-=1
          if alts<8:
            th=1000 '''      
       
       if th<1000:
         th=1000
       elif th>2000:
        th=2000          
       #print("th:",th,"markers:",markers)
                       
       cmd.rcThrottle=th
       
       if alts>10:
         stableflag=1
       
       if i==4:
         chase=1
         ki=3
         maxpropx=42
         maxpropy=45
         falt=24
         i+=1
         
       ################################################################
       ###### Special case if drone is crashing with the runner #######
       #################################################################
       if alts<10 and chase==1 and hoverflag==0 and abs(errorx)<1.5 and abs(errory)<1.5:
        #print("special case considered")
        temp=x
        x=fpxx
        fpxx=temp
        temp=y
        y=fpyy
        fpyy=temp
        
        errorx=fpxx-x
        errory=fpyy-y
        diffx=prex-x
        diffy=prey-y
        #diffx=errorx-errorprex
        #diffy=errory-errorprey
        if abs(errorx)<2.2:
                integralx+=errorx/100
        if abs(errory)<2.2: 
                integraly+=errory/100
         
       
        ##### To check the stablity of the drone on a fixed target position ######    
        if abs(errorx)<0.6 and abs(errory)<0.6 and stableflag==1:
                flag+=1
        else:
                flag=0 
        
       
        ###### Moving the drone with a consistant speed if drone is far away from the target  #######
         
        '''if abs(errorx)>=1.6:
                kpx/=1.3
                kdx*=0.7 
        if abs(errory)>=1.6:
                kpy/=1.3
                kdy*=0.7'''
        
       
        ##### If the marker is at corner positions then whycon is unable to detect the exact position of the marker ####
        ##### Hence the position of marker changes with +-1 even if marker is stabe , So such unstable noise values ####
        ##### are very vulnerable for derivative term of PID . Hence to avoid this a LOW PASS filter is applied to  ####
        ##### differece in values of linear data to remove the noise                                                ####
       
        if abs(x)>8: 
                difx[0]=difx[1]
                difx[1]=difx[2]
                difx[2]=diffx 
                diffx=difx[0]*0.3+difx[1]*0.3+difx[2]*0.4
      
        if abs(y)>6: 
                dify[0]=dify[1]
                dify[1]=dify[2]
                dify[2]=diffy 
                diffy=dify[0]*0.3+dify[1]*0.3+dify[2]*0.4
       
       
        ###### To bind the change in position values to upper and lower limit ######
       
        if diffx>0.34:
                diffx=0.34
        elif diffx<-0.34:
                diffx=-0.34  
        if diffy>0.34:
                diffy=0.34
        elif diffy<-0.34:
                diffy=-0.34 
       
       
       
        ##### When the drone marker is not drone_detected for a moment then the difference between the current and previous posoion #####
        ##### become high which is vulnerable for derivative term of the PID . Hence the normal referesh time is 0.04 and if  #####
        ##### if image drone_detected after a time more than 0.04 then the differece is devided by the time to get the real speed   ##### 
       
        diffx=(diffx*0.04)/(drone_timediff)
        diffy=(diffy*0.04)/(drone_timediff)
        drone_timediff=0.04
          
         
          
        ##### If the marker is not drone_detected then the current position of the drone is considered as the last detect position #####
        ##### but if drone was moving then the difference in speed is considred as previous so that Derivative term of PID   #####
        ##### can controll the drone motion even if drone marker is not detecting . This prevent the drone form going out    #####
        ##### from arena in some situations and making drone stable if marker is not drone_detected in some situations             #####
       
        if drone_detected==0:
                diffx=diffprex
                diffy=diffprey
       
        if chase==1:
                kpx=24
                kpy=23
                kdx*=0.9
                kdy*=0.9
       
        prox=kpx*errorx
        proy=kpy*errory
       
        #### To set the upper and lower bound on the proportional value so that drone move at consistant speed if target is far away ###
        factx=1
        facty=1
        if errorx!=0:
                factx=errorx/abs(errorx)
                rerrorx=abs(errorx/errory)
        if errory!=0: 
                facty=errory/abs(errory)
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
        speedx=prox+kin*integralx+kdx*(diffx)
        speedy=proy+kin*integraly+kdy*(diffy)
       
        ### To bound the maximum error ouput as +/- 69  ###
        if speedx>69:
                speedx=69
        elif speedx<-69:
                speedx=-69
        if speedy>69:
                speedy=69
        elif speedy<-69:
                speedy=-69
       
       
        ####*********************************************************************************************####
        ### The error calculated from Image cordinats is added with self balancing feedback loop values  #####
        ### The pitch is trimmed with -2 and rolls trimmed with 3 i.e. mid pitch val is 2 and mid roll  #####
        ### val is -3   The output is bounded with max 1600 and min 1400                                #####
        ### ********************************************************************************************#####
          
        valx=0.0
        valy=0.0
     
        ####assigning the variables to x and y direction  
       
       
        #print("valx:",speedx,"valy:",speedy)    
        valx=1500-speedx
        valy=1500-speedy
        #print("controlled")
       
        if valx>1600:
                valx=1600
        if valx<1400:
                valx=1400
        if valy>1600:
                valy=1600
        if valy<1400:
                valy=1400 
       
        ### Assigning the calculated values to global object variables  ###
        cmd.rcPitch=valx
        cmd.rcRoll=valy
       
    
       if runner_detected==0:
        cavecount+=1  
       
       
         
       ### Assigning the current values to global variables to use in the next itration
       rollpre=rolls
       pitchpre=pitchs
       prex=x
       prey=y
       
       prefpx=fpx
       prefpy=fpy
       
       precaveflag=caveflag
       
       errorprex=errorx
       errorprey=errory
       errorprez=errorz
       errorpreyaw=erroryaw
       
       diffprex=diffx
       diffprey=diffy
       
       markers=0
       drone_detected=0
       runner_detected=0
        


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
            
def listen(data):
     global fpz,fpx,fpy,kpx,kpy,kd,ki,seq,x,y,z,drone_timepre,runner_detect_counter,runner_haltx,runner_halty,drone_timediff,runner_timediff,runner_timepre,markers,prefpx,prefpy,drone_detected,runner_detected,cavecount,caveflag
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

def arm():
    cmd = PlutoMsg()
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1000
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1500
    move.publish(cmd)
    rospy.sleep(0.18)

'''
* Function Name: disarm
* Input:   None
* Output:  publish message to drone_command topic
* Logic:   publish message to drone_command topic for disarming the drone with required delay
*
* Example Call: disarm()
'''
    
def disarm():
    cmd = PlutoMsg()
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1300
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1200
    move.publish(cmd)
    rospy.sleep(1)  

'''
* Function Name: access_data
* Input:  floatar data (custom msg defined to get the values from drone)
* Output: save the pitch, roll, yaw in global variables   . Call the derive function according to the target and chase value 
* Logic:  the functions take floatar object from publisher and save the corresponding vlaues to pitchs ,rolls and yaws
          the first yaw value is stored in fyaw as fixed yaw reference  
          if chase value is 0 then chase follow the target according to the waypoint position , if the chase value is 1 then 
          target values are updating according to the runner position 
*
* Example Call: access_data(floatar)
'''

def access_data(data):
    global pitchs,yaws,rolls,alts,fyaw,kpx,kpy,ki,kdx,kdy,fpz,x,y,z,chase,i,wpx,wpy,yawflag
    pitchs=data.pitch
    rolls=data.roll
    yaws=data.yaw
    alts=data.alt
    if alts>7:
     yawflag=1
    if chase==0:
     #### Taking the target values from the array #####
     derive(x,y,z,kpx,kpy,ki,kdx,kdy,wpx[i],wpy[i],fpz,1)  
    else:
     #### The runner's marker is smaller then the chaser hence we multiply the runner value with the a factor f ##### 
     derive(x,y,z,kpx,kpy,ki,kdx,kdy,fpx*0.78,fpy*0.75,fpz,1)
    
    ####  To store the initial yaw as permanent yaw position during flight ####
    ####  i.e. the drone will remain in intial orientation during this flight only ####
    #if fyaw==0 and :
    '''if yawflag==0 and fyaw<yaws:                         
       fyaw=yaws  '''
    if fyaw==0:
     fyaw=yaws       
    #print("yaws:",fyaw);      

'''
* Function Name: talker
* Input: None
* Output: arm the drone, make instance of thread class , publish message to drone_command topic after a fixed interval
* Logic:  creating an instance of thread and starting it . Reading values from keyboard buffer(non blocking) and act 
          accordingly . Publish message to drone_command topic with a given delay i.e. 69 milliseconds
*
* Example Call: talker()
'''
     
def talker():
    global cmd,flag,kpx,kd,ki,errorx,errory ,fpx,fpy
    rospy.init_node('talker', anonymous=True)
    disarm()
    disarm()
    disarm()
    arm()
    rospy.sleep(0.18)
    arm()
    rospy.sleep(0.18)
    #### Making instance of thread and starting thread to receive the whycon and drone sensor values   #####
    threadpose=controlThread()
    threadpose.start()
    #threadtargets=targetThread()
    #threadtargets.start()

    while 1:
    
     ###########  Block to change the values of kp,ki and kd at runtime   #########
     while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
       line = sys.stdin.readline()
       if line=="e\n":         #to stop the node with disarming the drone if e is pressed followed by enter
        disarm()
        disarm()
        disarm()
        sys.exit() 
     else:
      time.sleep(0.11)         #delay to publish message to drone_command after every 90ms
      #print("x:",cmd.rcPitch,"y:",cmd.rcRoll,"H:",cmd.rcThrottle)
      #print("fpx:",fpx,"fpy:",fpy)
      move.publish(cmd)
      #derive(0,0,0,0,0,0,0,0,0,0,0,1)  
                 

        
'''
* Function Name: main
* Input: None
* Output: None
* Logic:  if python this file is executed on command line directly then __name__ is assigned __main__
          Hence from here the program starts executing
*
* Example Call: if condition is satisfied block starts executing
'''

if __name__ == '__main__':
    try:
       talker()
    except rospy.ROSInterruptException:
        pass
