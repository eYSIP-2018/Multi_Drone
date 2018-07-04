#!/usr/bin/env python
from plutodrone.srv import *
import rospy
from geometry_msgs.msg import PoseArray
from plutoserver.msg import floatar
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import threading

class request_data(threading.Thread):
	"""docstring for request_data"""
	def __init__(self,service,topic):
	        threading.Thread.__init__(self)
	        print(service)
	        self.drone_service=service
	        self.drone_topic=topic
	def run(self):        
		#rospy.init_node('drone_board_data')
		data = rospy.Service(self.drone_service, PlutoPilot, self.access_data)
		#magdata=rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.magdata=rospy.Publisher(self.drone_topic,floatar,queue_size=1)
		rospy.spin()

	def access_data(self, req):
		 #print "accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ)
		 #print "gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ)
		 print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		 print self.drone_topic + ":"
		 print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		 print "altitude = " +str(req.alt)
		 print " "
		 poses=floatar()
		 poses.pitch=req.accX
		 poses.roll=req.accY
		 poses.yaw=req.yaw
		 poses.alt=req.alt
                 self.magdata.publish(poses)
		 rospy.sleep(.1)
		 return PlutoPilotResponse(rcAUX2 =1500)

if __name__ == '__main__':
        rospy.init_node('drone_board_data')
        node_0 = request_data("PlutoService_0","/Sensor_data_0")
        node_1 = request_data("PlutoService_1","/Sensor_data_1")
        node_2 = request_data("PlutoService_2","/Sensor_data_2")
        node_0.start()
        node_1.start()
        #node_2.start()
        rospy.spin()


		
