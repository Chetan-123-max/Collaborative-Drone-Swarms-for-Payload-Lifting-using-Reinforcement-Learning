#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Int32
import rospy
from geometry_msgs.msg import Twist
import time
import math
import random
import numpy as np

def getReward(present_angle,next_angle):
	if(59<=next_angle<=61):
		return (300)     
	if(present_angle<=60):
		if((next_angle - present_angle)>0):
			return (100)
		else:
			return (-100)
	if(present_angle>60):
		if((next_angle - present_angle)>0):
			return (-100)
		else:
			return (100)



def code(angle):
	if(angle>=0 and angle<=3):
		state1=1
		return state1
	if(angle>3 and angle<=6):
		state2=2
		return state2
	if(angle>6 and angle<=9):
		state3=3
		return state3
	if(angle>9 and angle<=12):
		state4=4
		return state4
	if(angle>12 and angle<=15):
		state5=5
		return state5
	if(angle>15 and angle<=18):
		state6=6
		return state6
	if(angle>18 and angle<=21):
		state7=7
		return state7
	if(angle>21 and angle<=24):
		state8=8
		return state8
	if(angle>24 and angle<=27):
		state9=9
		return state9
	if(angle>27 and angle<=30):
		state10=10
		return state10
	if(angle>30 and angle<=33):
		state11=11
		return state11
	if(angle>33 and angle<=36):
		state12=12
		return state12
	if(angle>36 and angle<=39):
		state13=13
		return state13
	if(angle>39 and angle<=42):
		state14=14
		return state14
	if(angle>42 and angle<=45):
		state15=15
		return state15
	if(angle>45 and angle<=48):
		state16=16
		return state16
	if(angle>48 and angle<=51):
		state17=17
		return state17
	if(angle>51 and angle<=54):
		state18=18
		return state18
	if(angle>54 and angle<=57):
		state19=19
		return state19
	if(angle>57 and angle<=60):
		state20=20
		return state20
	if(angle>60 and angle<=63):
		state21=21
		return state21
	if(angle>63 and angle<=66):
		state22=22
		return state22
	if(angle>66 and angle<=69):
		state23=23
		return state23
	if(angle>69 and angle<=72):
		state24=24
		return state24
	if(angle>72 and angle<=75):
		state25=25
		return state25
	if(angle>75 and angle<=78):
		state26=26
		return state26
	if(angle>78 and angle<=81):
		state27=27
		return state27
	if(angle>81 and angle<=84):
		state28=28
		return state28
	if(angle>84 and angle<=87):
		state29=29
		return state29
	if(angle>87 and angle<=90):
		state30=30
		return state30
	if(angle>90 and angle<=93):
		state31=31
		return state31
	if(angle>93 and angle<=96):
		state32=32
		return state32
	if(angle>96 and angle<=99):
		state33=33
		return state33
	if(angle>99 and angle<=102):
		state34=34
		return state34
	if(angle>102 and angle<=105):
		state35=35
		return state35
	if(angle>105 and angle<=108):
		state36=36
		return state36
	if(angle>108 and angle<=111):
		state37=37
		return state37
	if(angle>111 and angle<=114):
		state38=38
		return state38
	if(angle>114 and angle<=117):
		state39=39
		return state39
	if(angle>117 and angle<=120):
		state40=40
		return state40
	if(angle>120 and angle<=123):
		state41=41
		return state41
	if(angle>123 and angle<=126):
		state42=42
		return state42
	if(angle>126 and angle<=129):
		state43=43
		return state43
	if(angle>129 and angle<=132):
		state44=44
		return state44
	if(angle>132 and angle<=135):
		state45=45
		return state45
	if(angle>135 and angle<=138):
		state46=46
		return state46
	if(angle>138 and angle<=141):
		state47=47
		return state47
	if(angle>141 and angle<=144):
		state48=48
		return state48
	if(angle>144 and angle<=147):
		state49=49
		return state49
	if(angle>147 and angle<=150):
		state50=50
		return state50
	if(angle>150 and angle<=153):
		state51=51
		return state51
	if(angle>153 and angle<=156):
		state52=52
		return state52
	if(angle>156 and angle<=159):
		state53=53
		return state53
	if(angle>159 and angle<=162):
		state54=54
		return state54
	if(angle>162 and angle<=165):
		state55=55
		return state55
	if(angle>165 and angle<=168):
		state56=56
		return state56
	if(angle>168 and angle<=171):
		state57=57
		return state57
	if(angle>171 and angle<=174):
		state58=58
		return state58
	if(angle>174 and angle<=177):
		state59=59
		return state59
	if(angle>177 and angle<=180):
		state60=60
		return state60
	

class agent():
	def __init__(self):
		rospy.init_node('bang')
		
		self.cmd1=Twist()
		self.pose_1=PoseStamped()
		
		self.drone_position_1=[0.0,0.5,0]
		self.setpoint_1 = [0.0,0.0,5.0]
		self.sample_time = 0.500

		self.Kp = [275,275,275]
		self.Ki = [1,1,1]
		self.Kd = [100,100,100]	
		
		self.error_1=[0,0,0]               
		self.error_sum_1=[0,0,0]           
		self.error_rate_1=[0,0,0]
		self.previous_error_1=[0,0,0] 
		self.out_1=[0,0,0]

		self.pub_string='/drone1/cmd_vel'
		self.subscribe_string='/drone1/ground_truth_to_tf/pose'
		print('confirming drone 1')

		self.pub1 = rospy.Publisher(self.pub_string, Twist, queue_size = 10)
		rospy.Subscriber(self.subscribe_string,PoseStamped,self.extract_1)
		
	def pose_cb(self, msg):
        	self.pose = msg

	def extract_1(self,msg):
		self.pose_1=msg
		self.drone_position_1[0] = self.pose_1.pose.position.x
		self.drone_position_1[1] = self.pose_1.pose.position.y
		self.drone_position_1[2] = self.pose_1.pose.position.z

	
	def lengthSquare(self,X, Y):  
		xDiff = X[0] - Y[0]  
		yDiff = X[1] - Y[1]  
		zDiff = X[2] - Y[2]
		return xDiff * xDiff + yDiff * yDiff + zDiff * zDiff
     
	def getAngle(self,B,A=(-5,0,5), C=(5,0,5)):  
       
		a2 = self.lengthSquare(B, C)  
		b2 = self.lengthSquare(A, C)  
		c2 = self.lengthSquare(A, B)  
		try:   
			a = math.sqrt(a2);  
			b = math.sqrt(b2);  
			c = math.sqrt(c2);  
   
			alpha = math.acos((b2 + c2 - a2) /(2 * b * c));  
			betta = math.acos((a2 + c2 - b2) /(2 * a * c));  
			gamma = math.acos((a2 + b2 - c2) /(2 * a * b));  
 
			alpha = alpha * 180 / math.pi;  
			betta = betta * 180 / math.pi;  
			gamma = gamma * 180 / math.pi;  
			#print('beta',betta) 
	
			return(round(betta))
		except ValueError:
			return(random.randrange(0,180))

	def calc_setpoint(self):
		self.setpoint_1[0]=((-5)+(5)+self.drone_position_1[0])/3.0
		self.setpoint_1[1]=((0)+(0)+self.drone_position_1[1])/3.0
		self.setpoint_1[2]=((5)+(5)+self.drone_position_1[2])/3.0
	def pid(self,action_index):
		self.error_1 = [dp - setp for dp,setp in zip(self.drone_position_1, self.setpoint_1)]
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
		if(action_index==1):
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
	def __init__(self, learning_rate=0.7, discount=0.95, exploration_rate=1.0, iterations=100):
		self.q_table = np.zeros((61,2))
		self.learning_rate = learning_rate
		self.discount = discount
		self.exploration_rate = 0.3
		self.exploration_delta = 1.0 / 20000

	def get_next_action(self, state):
		if random.random() < self.exploration_rate:
			print('exploring')
			return self.random_action()
		else:
			print('exploiting')
			return self.greedy_action(state)
	def greedy_action(self, state): #0-l    1-up    2-r    3-d
		g_action = np.argmax((self.q_table[state][0],self.q_table[state][1]))
		return g_action
	def random_action(self):
		return random.choice([0,1])

	def update(self, old_state, new_state, action, reward):
		old_value = self.q_table[old_state][action]
		future_action = self.greedy_action(new_state)
		future_reward = self.q_table[new_state][future_action]
		new_value = old_value*(1-self.learning_rate) + self.learning_rate * (reward + self.discount * future_reward)
		self.q_table[old_state][action] = new_value
		if self.exploration_rate > 0:
			self.exploration_rate -= self.exploration_delta/2

def initial_step(drone,initial_setpoint):
		drone.error_1 = [dp - setp for dp,setp in zip(drone.drone_position_1, initial_setpoint)]
		n_sum_1 = [a + b for a, b in zip(drone.error_1, drone.error_sum_1)]
		n_derr_1 = [a - b for a, b in zip(drone.error_1, drone.previous_error_1)] 
		out_vals_1=[sum(p*q for p, q in zip(a, b)) for a, b in zip(zip(drone.error_1,n_sum_1, n_derr_1), zip(drone.Kp, drone.Ki, drone.Kd))]
		drone.out_1[0]=-out_vals_1[0]   # Set Roll output value to average
		drone.out_1[1]=-out_vals_1[1]   # Set Pitch output value to average
		drone.out_1[2]=-out_vals_1[2]   # Set thrust output value to average
		drone.cmd1.linear.x=drone.out_1[0]/2000
		drone.cmd1.linear.y=drone.out_1[1]/2000
		drone.cmd1.linear.z=drone.out_1[2]/2000	
		
		drone.error_sum_1=drone.error_sum_1+drone.error_1	
		drone.previous_error_1=drone.error_1
		drone.pub1.publish(drone.cmd1)
		time.sleep(drone.sample_time)

if ('__main__'==__name__):
	stock=0
	learning_rate = 0.7
    	discount = 0.95
    	iterations = 10000
	drone=agent()
	Q=Q()
	Q.q_table=np.load('try6_Q_table_2.npy')
	print(Q.q_table)
	initial_setpoint=[drone.drone_position_1[0],drone.drone_position_1[1],5.0]
	while(not(5.0< drone.drone_position_1[2] <5.10)):
		initial_step(drone,initial_setpoint)
		time.sleep(drone.sample_time)
	print(drone.drone_position_1,' reached the required plane')
	drone.calc_setpoint()
	drone.pid_reset()
	while not rospy.is_shutdown():
		for step in range(iterations*10):
			present_angle=drone.getAngle(drone.drone_position_1)
			present_state=code(present_angle)
			present_action=Q.get_next_action(present_state)
			print('present state angle: ',present_angle)
			while(True):
				drone.pid(present_action)
				cur_pos=drone.drone_position_1
				new_angle=drone.getAngle(cur_pos)
				angle_diff=abs(new_angle - present_angle)
				#print('angle diff : ',angle_diff)
				if(angle_diff==3):
					next_state=code(new_angle)
					print('next state angle:',new_angle)
					break
			reward=getReward(present_angle,new_angle)
			Q.update(present_state,next_state,present_action,reward)
			print('Present State:',present_state,' nextstate:',next_state,' present_action:',present_action,' present reward:',reward)
			np.save('try6_Q_table_2.npy',Q.q_table)
			Q.q_table=np.load('try6_Q_table_2.npy')
		np.save('final_try6_Q_table_2.npy',Q.q_table)
		np.load('final_try6_Q_table_2.npy')						
		print('Q-Table saved !!')
