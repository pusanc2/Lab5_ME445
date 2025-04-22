#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	M = np.array([
        [0, -1, 0, 0.39],
        [0, 0, -1, 0.401],
        [1, 0, 0, 0.2155],
        [0, 0, 0, 1]
     ])
	w_list = np.array([
	[0, 0, 1],
	[0, 1, 0],
	[0, 1, 0],
	[0, 1, 0],
	[1, 0, 0],
	[0, 1, 0]
	])

	q_list = np.array([
	[-0.15, 0.15, 0],
	[-0.15, 0, 0.162],
	[0.094, 0, 0.162],
	[0.307, 0, 0.162],
	[0, 0.260, 0.162],
	[0.390, 0, 0.162]
	])

	S = np.column_stack([np.concatenate((w, np.cross(-w, q))) for w, q in zip(w_list, q_list)])

	print(M, "\n")
	print(S, "\n")

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()


	T = np.eye(4)
 
	for i in range(6):

		# Extract w,v
		w = S[:3, i]
		v = S[3:, i]
	
		# Build S matrix
		S_matrix = np.zeros((4, 4))
		S_matrix[0:3, 0:3] = np.array([
			[0, -w[2], w[1]],
			[w[2], 0, -w[0]],
			[-w[1], w[0], 0]
		])
		S_matrix[0:3, 3] = v
	
		# Matrix exponential
		T_i = expm(S_matrix * theta[i])
	
		T = T @ T_i
	
	T = T @ M

	np.set_printoptions(suppress=True, precision=8)

	# ==============================================================#

	print(str(T) + "\n")
	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	L1 = 0.152
	L2 = 0.120
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059
	al_plate = 0.027

	#deg to rad convert
	yaw = np.deg2rad(yaw_WgripDegree)
 
	# coordinate conversion
	xgrip = xWgrip + 0.15
	ygrip = yWgrip - 0.15
	zgrip = zWgrip - 0.01
	
	xcenter = xgrip - L9 * np.cos(yaw)
	ycenter = ygrip - L9 * np.sin(yaw)
	zcenter = zgrip
	 
	# theta 5
	theta5 = -np.pi / 2
 
 	# theta1
	hyp = (np.sqrt(xcenter**2 + ycenter**2))
	theta_small = np.arcsin((L6 + al_plate) / hyp)
	theta_large = np.arctan2(ycenter, xcenter)
	theta1 = theta_large - theta_small
 
	#theta 6 
	theta6 = theta1 + np.pi / 2 - yaw 	
 
	#projected end point
	z3end = zcenter + L10 + L8
	x3end = ((hyp * np.cos(theta_small)- L7) * np.cos(theta1)) 
	y3end = ((hyp * np.sin(theta_small)- L7) * np.sin(theta1))
 
	#theta3
	zoffset = z3end - L1
	r = np.sqrt(x3end**2 + y3end**2)
	h = np.sqrt(r**2 + zoffset**2)
	a = np.arccos((L5**2 + L3**2 - h**2)/(2 * L5 * L3))
	theta3 = np.pi - a

	#theta2
	theta2a = np.arccos((L3**2 + h**2 - L5**2) / (2 * L3 * h))
	theta2b = np.arctan2(zoffset,r)
	theta2 = -(theta2a + theta2b)

	#theta4
	theta4 = -(theta2 + theta3)
  
	print("theta1: ", theta1 * 180 / np.pi, "\ntheta2: ", theta2 * 180 / np.pi, 
       "\ntheta3: ", theta3 * 180 / np.pi, "\ntheta4: ", theta4 * 180 / np.pi, 
       "\ntheta5: ", theta5 * 180 / np.pi, "\ntheta6: ", theta6 * 180 / np.pi)

	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
