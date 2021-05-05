#!/usr/bin/env python
import math
import cmath
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import geometry_msgs.msg
import tf

pi = math.pi

rospy.init_node('delta_robot') # Node initialization in ROS
broadcaster = tf.TransformBroadcaster()
odom_trans = geometry_msgs.msg.TransformStamped()
joint_state = JointState()
odom_trans.header.frame_id = 'odom'
odom_trans.child_frame_id = 'base'
joint_state.header = Header()
joint_state.name = ['base_to_upper_arm_1', 'base_to_upper_arm_2', 'base_to_upper_arm_3', 'upper_to_elbow_1', 'upper_to_elbow_2', 'upper_to_elbow_3', "elbow_to_lower_1_A", "elbow_to_lower_2_A", "elbow_to_lower_3_A", "elbow_to_lower_1_B", "elbow_to_lower_2_B", "elbow_to_lower_3_B", 'end_effector_joint', "lower_arm_1_A_to_lower_elbow"]

# Robot properties
L = 0.350  # Upper arms length
l = 0.800  # Lower legs length
sb = 0.8 * math.sin(120 * pi/360)		# base equilateral triangle side
wb = (math.sqrt(3)/6) * sb # 0.180; planar distance from {0} to near base side
ub = (math.sqrt(3)/3) * sb # 0.360; planar distance from {0} to a base vertex
sp = 0.096 * math.sin(120 * pi/360)	# triangle side of end-effector
wp = (math.sqrt(3)/6) * sp # planar distance from {P} to near platform side
up = (math.sqrt(3)/3) * sp # planar distance from {P} to a platform vertex

# reserve storage and define size/shape for the most used variables
E = np.zeros(3) # Variable for calculated angles 
F = np.zeros(3) # Variable for calculated angles 
G = np.zeros(3) # Variable for calculated angles

t1 = np.zeros(3) # Variable for calculated angle
t2 = np.zeros(3) # Variable for calculated angle

# in general there are two possible solutions 
angle_upper_arm_rad_1 = np.zeros(3) # First solution of motor angle in radian
angle_upper_arm_rad_2 = np.zeros(3) # Second solution of motor angle in radian
angle_upper_arm_deg_1 = np.zeros(3) # First solution of motor angle in degree
angle_upper_arm_deg_2 = np.zeros(3) # Second solution of motor angle in degree

# solution for the lower arm dependend on the calculated and coosen angle_upper_arm 
final_angle_lower_arm1 = np.zeros(1)
final_angle_lower_arm2 = np.zeros(1)
final_angle_lower_arm3 = np.zeros(1)

end_effector = np.zeros(1) # angle between l and e_e_platform -> assumption for the delta robot: 0 deg in regular COS

k=0 # Flag for changing coordinates


# Equations used from Kinematic_Calculation_IK_FK_DeltaRobot.pdf

def calculate_delta_robot_angle(x, y, z):
#Calculate a, b, c
	a = wb - up
	b = (sp/2.0) - ((math.sqrt(3.0)/2.0) * wb)
	c = wp - (wb/2.0)
#Calculate E[0], F[0], G[0]
	E[0] = 2.0 * L * (y + a)
	F[0] = 2.0 * z * L
	G[0] = math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(a, 2) + math.pow(L, 2) + (2.0 * y * a) - math.pow(l, 2)
#Calculate E[1], F[1], G[1]
	E[1] = -L * ((math.sqrt(3.0) * (x + b)) + y + c)
	F[1] = 2.0 * z * L
	G[1] = math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c, 2) + math.pow(L, 2) + (2.0 * ((x * b)+ (y * c))) - math.pow(l, 2)
#Calculate E[2], F[2], G[2]
	E[2] = L * ((math.sqrt(3.0) * (x - b)) - y - c)
	F[2] = 2.0 * z * L
	G[2] = math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c, 2) + math.pow(L, 2) + (2.0 * (-(x * b) + (y * c))) - math.pow(l, 2)

#Calculate angle in rad and degrees
	for i in range(0, 3):
		t1[i] = (-F[i] + math.sqrt(math.pow(F[i],2) - math.pow(G[i],2) + math.pow(E[i],2)))/(G[i] - E[i])
		t2[i] = (-F[i] - math.sqrt(math.pow(F[i],2) - math.pow(G[i],2) + math.pow(E[i],2)))/(G[i] - E[i])
		angle_upper_arm_rad_1[i] = 2 * math.atan(t1[i])
		angle_upper_arm_rad_2[i] = 2 * math.atan(t2[i])
		angle_upper_arm_deg_1[i] = math.degrees(angle_upper_arm_rad_1[i])
		angle_upper_arm_deg_2[i] = math.degrees(angle_upper_arm_rad_2[i])


def calculate_angle_lower_arm(x, y, z, phi, theta1):
	# calculation based on Delta_robot_Inverse_direct_and_intermediate_Jacobi.pdf
	theta3 = math.acos((y * math.sin(phi) + x * math.cos(phi))/l)
	#print("theta1:", theta1)
	#print("theta3:", theta3)
	angle = (z + L * math.sin(theta1))/(l * math.sin(theta3))
	theta2 = math.asin(angle) - theta1 +  pi/2 # asin is defined from -1 to 1
	
	return theta2, theta3

def calculate_lower_arms(x, y, z, theta_a):
	theta_b = np.zeros(3) # array for the vertical angles
	theta_c = np.zeros(3) # array for the horizontal angles
	
	for i in range(3):
		phi = ((i * 120) * pi)/180
		theta_b[i], theta_c[i] = calculate_angle_lower_arm(x_, y_, z, phi, theta_a[i])

	return theta_b, theta_c


def send_joint_state_to_urdf():
#sending joint state to URDF
	joint_state.header.stamp = rospy.Time.now()
	joint_state.position = [upper_arm_1, upper_arm_2, upper_arm_3, upper_to_elbow_1, upper_to_elbow_2, upper_to_elbow_3, elbow_to_lower_1_A, elbow_to_lower_2_A, elbow_to_lower_3_A, elbow_to_lower_1_B, elbow_to_lower_2_B, elbow_to_lower_3_B, end_effector_joint, lower_arm_1_A_to_lower_elbow]
	odom_trans.header.stamp = rospy.Time.now()
	pub.publish(joint_state)
	broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(),'base','odom')
	
#Define Publisher
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

# Main Program
r = rospy.Rate(1)
while not rospy.is_shutdown():
# Loop the coordinates
	if k == 0:
		x = 0.0
		y = 0.0
		z = -0.43 # max -0.43
	if k == 1:
		x = 0.1
		y = 0
		z = -0.55
	if k == 2:
		x = 0.2
		y = 0
		z = -0.75
	if k == 3:
		x = 0.3
		y = 0
		z = -1.0
	

	def transform_coordinates(x, y, spin):
		x_ = x * math.cos(spin) - y * math.sin(spin)
		y_ = x * math.sin(spin) + y * math.cos(spin)

		return x_, y_

	spin = -210
	x_, y_ = transform_coordinates(x, y, spin)  
	# required to calculate the right position, the calculations are turning the coordinates
	
	calculate_delta_robot_angle(x_, y_, z)
	upper_arm_1 = angle_upper_arm_rad_2[0] 
	upper_arm_2 = angle_upper_arm_rad_2[1]
	upper_arm_3 = angle_upper_arm_rad_2[2]

	theta_a =[upper_arm_1, upper_arm_2, upper_arm_3] 
	theta_b, theta_c = calculate_lower_arms(x_, y_, z, theta_a)

	# define the angles for publishing
	upper_to_elbow_1 = theta_b[0] 
	upper_to_elbow_2 = theta_b[1] 
	upper_to_elbow_3 = theta_b[2] 
	elbow_to_lower_1_A = theta_c[0] - (pi/2)
	elbow_to_lower_2_A = theta_c[1] - (pi/2)
	elbow_to_lower_3_A = theta_c[2] - (pi/2)

	lower_arm_1_A_to_lower_elbow = theta_c[0] - (pi/2)
		
	elbow_to_lower_1_B = theta_c[0] - (pi/2)
	elbow_to_lower_2_B = theta_c[1] - (pi/2)
	elbow_to_lower_3_B = theta_c[2] - (pi/2)
	
	end_effector_joint = (angle_upper_arm_rad_2[0] + upper_to_elbow_1) * -1

	send_joint_state_to_urdf() 

	k=k+1
	if k == 4:
		k = 0
	
	print(angle_upper_arm_deg_2)
	print(angle_upper_arm_rad_2)
	
	r.sleep()

