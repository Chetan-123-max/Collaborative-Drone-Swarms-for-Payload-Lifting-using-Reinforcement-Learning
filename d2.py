'''
Necessary Imports
'''
#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Int32
import rospy
from geometry_msgs.msg import Twist
import time
import math
import random
import numpy as np

#########################################################

'''
Derive the state based on the angle at corresponding vertex
'''
def get_state(angle):
	if(angle == 0):
		return(1)
	return(math.ceil(angle/3))
	
#########################################################
'''
Object class for corresponding drone
'''
class agent():
	def __init__(self):
		rospy.init_node('drone1')
		self.cmd1=Twist()		#  Message
		self.pose_1=PoseStamped()       #  Types
		self.check = Int32()		#
		
		self.alpha=0			#  Angles
		self.gamma=0			#  of the
		self.betta=0			# triangle
		
		self.drone_position_1=[10.0,10.0,0.0]
		self.drone_position_2=[0.0,10.0,0.0]
		self.drone_position_3=[10.0,0.0,0.0]
		self.setpoint_1 = [0.0,0.0,5.0]  
		self.sample_time = 0.500

		self.Kp = [275,275,275]		#  PID
		self.Ki = [1,1,1]		#  Gain 
		self.Kd = [100,100,100]		#  Constants
		
		self.rel_d = [0,0,0]  		# Holds relative distances
		
		self.error_1=[0,0,0]               
		self.error_sum_1=[0,0,0]           
		self.error_rate_1=[0,0,0]
		self.previous_error_1=[0,0,0] 
		self.out_1=[0,0,0]

		self.pub_string='/drone1/cmd_vel'
		self.subscribe_string_1='/drone1/ground_truth_to_tf/pose'
		self.subscribe_string_2='/drone2/ground_truth_to_tf/pose'
		self.subscribe_string_3='/drone3/ground_truth_to_tf/pose'
		print('confirming drone 1')

		self.pub1 = rospy.Publisher(self.pub_string, Twist, queue_size = 10)
		self.pub2 = rospy .Publisher('check_value',Int32, queue_size = 1)	
		rospy.Subscriber(self.subscribe_string_1,PoseStamped,self.extract_1)	#  Position  
		rospy.Subscriber(self.subscribe_string_2,PoseStamped,self.extract_2)	#  Data
		rospy.Subscriber(self.subscribe_string_3,PoseStamped,self.extract_3)	#  Topics
		rospy.Subscriber('/check_value', Int32, self.pose_cb)
	
	'''
	Callback functions
	'''
	def pose_cb(self, msg):
        	self.pose = msg
        	self.check = self.pose.data

	def extract_1(self,msg):
		self.pose_1=msg
		self.drone_position_1[0] = self.pose_1.pose.position.x
		self.drone_position_1[1] = self.pose_1.pose.position.y
		self.drone_position_1[2] = self.pose_1.pose.position.z

	def extract_2(self,msg):
		self.pose_2=msg
		self.drone_position_2[0]=self.pose_2.pose.position.x
		self.drone_position_2[1]=self.pose_2.pose.position.y
		self.drone_position_2[2]=self.pose_3.pose.position.z

	def extract_3(self,msg):
		self.pose_3=msg
		self.drone_position_3[0]=self.pose_3.pose.position.x
		self.drone_position_3[1]=self.pose_3.pose.position.y
		self.drone_position_3[2]=self.pose_3.pose.position.z

	def lengthSquare(self,X, Y):  
		xDiff = X[0] - Y[0]  
		yDiff = X[1] - Y[1]  
		zDiff = X[2] - Y[2]
		return xDiff * xDiff + yDiff * yDiff + zDiff * zDiff
		
	def get_d(self):
		self.rel_d[0]=math.sqrt((self.drone_position_1[0]-self.drone_position_2[0])**2+(self.drone_position_1[1]-self.drone_position_2[1])**2+(self.drone_position_1[2]-self.drone_position_2[2])**2)
		self.rel_d[1]=math.sqrt((self.drone_position_1[0]-self.drone_position_3[0])**2+(self.drone_position_1[1]-self.drone_position_3[1])**2+(self.drone_position_1[2]-self.drone_position_3[2])**2)
		self.rel_d[2]=math.sqrt((self.drone_position_2[0]-self.drone_position_3[0])**2+(self.drone_position_2[1]-self.drone_position_3[1])**2+(self.drone_position_2[2]-self.drone_position_3[2])**2)
     
	def getAngle(self,A):
       		B=self.drone_position_1
		C=self.drone_position_3
		a2 = self.lengthSquare(B, C)  
		b2 = self.lengthSquare(A, C)  
		c2 = self.lengthSquare(A, B)  
		try:   
			a = math.sqrt(a2);  
			b = math.sqrt(b2);  
			c = math.sqrt(c2);  
   
			self.alpha = math.acos((b2 + c2 - a2) /(2 * b * c));  
			self.betta = math.acos((a2 + c2 - b2) /(2 * a * c));  
			self.gamma = math.acos((a2 + b2 - c2) /(2 * a * b));  
 
			self.alpha = self.alpha * 180 / math.pi;  
			self.betta = self.betta * 180 / math.pi;  
			self.gamma = self.gamma * 180 / math.pi;  

			return(round(self.alpha))
		except ValueError:
			return(random.randrange(0,180))

	def calc_setpoint(self):   # Derives centroid location
		self.setpoint_1[0]=(self.drone_position_1[0]+self.drone_position_2[0]+self.drone_position_3[0])/3.0
		self.setpoint_1[1]=(self.drone_position_1[1]+self.drone_position_2[1]+self.drone_position_3[1])/3.0
		self.setpoint_1[2]=(self.drone_position_1[2]+self.drone_position_2[2]+self.drone_position_3[2])/3.0
		
	def pid(self,action_index):
		self.error_1 = [dp - setp for dp,setp in zip(self.drone_position_2, self.setpoint_1)]
		n_sum_1 = [a + b for a, b in zip(self.error_1, self.error_sum_1)]
		n_derr_1 = [a - b for a, b in zip(self.error_1, self.previous_error_1)] 
		out_vals_1=[sum(p*q for p, q in zip(a, b)) for a, b in zip(zip(self.error_1,n_sum_1, n_derr_1), zip(self.Kp, self.Ki, self.Kd))]
		self.out_1[0]=-out_vals_1[0]   # Set Roll output value to average
		self.out_1[1]=-out_vals_1[1]   # Set Pitch output value to average
		self.out_1[2]=-out_vals_1[2]   # Set thrust output value to average
		
		if(action_index==0):  #move towards
			self.cmd1.linear.x=self.out_1[0]/2000
			self.cmd1.linear.y=self.out_1[1]/2000
			self.cmd1.linear.z=self.out_1[2]/2000	
		if(action_index==1):  #move away
			self.cmd1.linear.x=-self.out_1[0]/2000
			self.cmd1.linear.y=-self.out_1[1]/2000
			self.cmd1.linear.z=-self.out_1[2]/2000	
		self.error_sum_1=self.error_sum_1+self.error_1	
		self.previous_error_1=self.error_1
		self.pub1.publish(self.cmd1)
		
	def pid_reset(self):
		self.error_sum_1=[0,0,0]
		self.previous_error_1=[0,0,0]

			
class Q():
	def __init__(self):
		self.q_table = np.load('try6_Q_table_2.npy')  		# Load Q table
		
	def get_next_action(self, state):
		return self.greedy_action(state)
	def greedy_action(self, state):
		g_action = np.argmax((self.q_table[state][0],self.q_table[state][1]))
		return g_action


def initial_step(drone,initial_setpoint):
		drone.error_1 = [dp - setp for dp,setp in zip(drone.drone_position_2, initial_setpoint)]
		n_sum_1 = [a + b for a, b in zip(drone.error_1, drone.error_sum_1)]
		n_derr_1 = [a - b for a, b in zip(drone.error_1, drone.previous_error_1)] 
		out_vals_1=[sum(p*q for p, q in zip(a, b)) for a, b in zip(zip(drone.error_1,n_sum_1, n_derr_1), zip(drone.Kp,drone.Ki, drone.Kd))]
		drone.out_1[0]=-out_vals_1[0]   # Set Roll output value
		drone.out_1[1]=-out_vals_1[1]   # Set Pitch output value 
		drone.out_1[2]=-out_vals_1[2]   # Set thrust output value
		drone.cmd1.linear.x=drone.out_1[0]/2000
		drone.cmd1.linear.y=drone.out_1[1]/2000
		drone.cmd1.linear.z=drone.out_1[2]/2000
		
		drone.error_sum_1=drone.error_sum_1+drone.error_1	#Update error sum	
		drone.previous_error_1=drone.error_1			#Update new error
		drone.pub1.publish(drone.cmd1)
		time.sleep(drone.sample_time)


if ('__main__'==__name__):
	drone=agent()
	drone.pub2.publish(1)
	print('This is ',drone.check)
	Q=Q()
	print(Q.q_table)
	initial_setpoint=[drone.drone_position_2[0],drone.drone_position_2[1],5.0]
	
	'''
	Initial hovering to required plane
	'''
	while(not(4.7< drone.drone_position_2[2] <5.10)):
		
		initial_step(drone,initial_setpoint)
		time.sleep(drone.sample_time)
	print(drone.drone_position_2,' reached the required plane')
	drone.calc_setpoint()
	drone.pid_reset()
	present_angle=drone.getAngle(drone.drone_position_2)
	present_state=get_state(present_angle)
	c = 0
	while not rospy.is_shutdown():
		if(drone.check == 2):		# System formed, proceed to new location in same orientation
			c = 1
			drone.pub2.publish(2)
			print('This is ',drone.check)
			print('The triangle is formed!')
			print('Now moving the strucure to a new coordinate')
			new_cen_coo=[10,15,5]   # New location coordinates
			drone.calc_setpoint
			present_cen_coo=drone.setpoint_1
			delta_x=new_cen_coo[0]-present_cen_coo[0]
			delta_y=new_cen_coo[1]-present_cen_coo[1]
			delta_z=new_cen_coo[2]-present_cen_coo[2]
			delta = [delta_x, delta_y, delta_z]
			new_setpoint = [a + b for a, b in zip(drone.drone_position_2, delta)]
			while(True):
				initial_step(drone,new_setpoint)
				time.sleep(drone.sample_time)

	
		drone.get_d()
		print('CHeck is',drone.check)
		
		'''
		Form an Equiateral Triangle
		'''
		if(not(c) and not ((57<=drone.alpha<=63) and (57<=drone.betta<=63) and (57<=drone.gamma<=63) and(drone.rel_d[0]<8) and(drone.rel_d[1]<8) and(drone.rel_d[2]<8))):
			present_action=Q.get_next_action(present_state)
			print('present state angle: ',present_angle)
			while(True):
				drone.pid(present_action)
				cur_pos=drone.drone_position_2
				new_angle=drone.getAngle(cur_pos)
				angle_diff=abs(new_angle - present_angle)
				if(angle_diff==3):
					next_state=get_state(new_angle)
					break
			present_state=next_state
			present_angle=new_angle
		
		else:				# Update flag value
			drone.pub2.publish(2)


