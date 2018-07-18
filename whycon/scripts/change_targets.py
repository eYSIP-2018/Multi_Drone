#!/usr/bin/env python

import sys
import rospy
import readchar
import threading
import select
from whycon.srv import *
from geometry_msgs.msg import  PoseArray

#### GLOABAL VAIRABLE  ##########
seq=0
preseq=0

######### Function to request the whycon Reset Service  ####################
def set_targets(x):
  rospy.wait_for_service('/whycon/reset')
  try:
        targets=rospy.ServiceProxy('/whycon/reset',SetNumberOfTargets)
        obj=targets(x)
  except rospy.ServiceException, e:
        print "Service call failed: %s"%e


###############  Thread To update the seq of the markers  ################
class controlThread(threading.Thread):
    
    def run(self):
     print("calling")
     rospy.Subscriber('/whycon/poses',PoseArray,listen)
     rospy.spin()        

def listen(data):
   global seq
   seq=data.header.seq
   #print(seq)
        
         
if __name__=="__main__":
        global seq,preseq
        rospy.init_node('target_ch', anonymous=True)
        read_seq=controlThread()
        read_seq.start()
        set_targets(2)
        while 1:
          while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
            line = sys.stdin.readline()
            if line=="e\n":         #to stop the node with disarming the drone if e is pressed followed by enter
             sys.exit()
             
          rospy.sleep(0.15)   
          if seq==preseq:                
           set_targets(1)
           rospy.sleep(0.6)
           set_targets(2) 
          preseq=seq 
          
          
          
          
          
                       
