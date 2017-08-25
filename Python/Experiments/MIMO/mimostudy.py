'''

Use this script from terminal / console with 
./python sisostudy.py --file_storage=my_runs

Will create an output with all the necessary information
'''

# Import the pacakges
# Numpy for numerical methods
import numpy as np 
# Python Control for SISO creation etc.
import control as cn
# Pandas for Data Storage
import pandas as pd
# Import the Algorithms
import sys
sys.path.append('../../')
import Algorithms as alg

import sys


# Define an experiment
from sacred import Experiment
ex = Experiment()

# Configuration
@ex.config
def experimental_setup():
	# Set up the Experiment and define the range of system gain, lag and delay as well as filename etc
	filename = 'MIMO_24082017_1.csv'
	# Sample size per system order
	sample_size = 1
	# Size, assuming quadratic system
	sys_size = 2
	# Maximum System Order (-1)
	max_order = 8
	# System noise in percent of gain
	noise_limit = 0
	# Gain Limits
	gain_limits = [0.0,10]
	# Lag Limits 
	lag_limits = [1,np.sqrt(200**(1/max_order))]
	# Delay Limits -> If small no delay
	delay_limits = [1e-2, 1e-3]
	# Create the system numerator
	N = np.random.uniform(gain_limits[0],gain_limits[1],(max_order,sample_size,sys_size,sys_size))
	# Create the system denominator -> Several
	D = np.random.uniform(lag_limits[0],lag_limits[1],(max_order,sample_size,max_order,sys_size,sys_size))
	# Create the system delay
	L = np.random.uniform(delay_limits[0], delay_limits[1],(max_order,sample_size))
	# Create an array for the results: sys_no, order, K,T, k,t,l, ms_real,ms_ideal, 4x t_rise, mp, t_settle, yss 
	columns = ['Sample No.', 'Order', 'K', 'TSum', 'KM', 'TM', 'LM', 'MS Real', 'MS Ideal', 'Tr_RSW', 'Mp_RSW', 'Ts_RSW', 'Ys_RSW', 'Tr_R', 'Mp_R', 'Ts_R', 'Ys_R', 'Tr_ISW', 'Mp_ISW', 'Ts_ISW', 'Ys_ISW', 'Tr_I', 'Mp_I', 'Ts_I', 'Ys_I']
	R = pd.DataFrame(columns=columns)

# Experimental Study
@ex.automain
def experiment(N,D,L,R,noise_limit,sys_size,sample_size,max_order,filename, columns):
	# Define system no
	sys_no = 0

	# Define an outer loop over system order
	for order in range(1,max_order):
		# Define an inner loop over samples
		for sample in range(0,sample_size):
			print(order,sample)
			# Current System
			sys_no += 1
			# Create the MiMo System
			# Numerator Preallocate 
			# FIRST MATRIX HAS TO BE EYE as list
			num = np.eye(sys_size)
			num = num[:,:,np.newaxis]
			num = num.tolist()
			# Denominator Preallocate
			den = np.ones((sys_size,sys_size,1)).tolist()
			# Make a static gain system
			G = cn.tf(num,den)
			
			# CHECK

			# Numerator Preallocate
			num = np.ones((sys_size,sys_size,1)).tolist()
			# Denominator Preallocate for all coefficients
			den1 = np.zeros((sys_size,sys_size,2))
			den2 = np.zeros((sys_size,sys_size,2))
			# Get the p^0 coefficient equal to 1
			for outputs in range(0,sys_size):
				for inputs in range(0,sys_size):
					den1[outputs][inputs][-1] = 1
					den2[outputs][inputs][-1] = 1

			den1 = den1.tolist()
			den2 = den2.tolist()
			
			# CHECK

			# Loop over the order
			for current_order in range(1,order+1):
				for outputs in range(0,sys_size):
					for inputs in range(0,sys_size):
						if current_order == 1:
							den1[outputs][inputs][-2] = D[order][sample][current_order][outputs][inputs]
							
						else:  
							# Get the current first order system
							den2[outputs][inputs][-2] = D[order][sample][current_order][outputs][inputs]
							den1[outputs][inputs] = np.convolve(den1[outputs][inputs],den2[outputs][inputs])
							
						# Get the system gain for last order
						if current_order >= order:
							num[outputs][inputs][0] = N[order][sample][outputs][inputs]
				# Make the MIMO System
			G = cn.tf(num,den1)
			G = cn.tf2ss(G)
			#G = cn.matlab.rss(states = order, inputs = sys_size, outputs= sys_size)
			#print(G)
			# Preallocate the Parameter
			K = np.zeros((sys_size,sys_size))
			T = np.zeros((sys_size,sys_size))
			L = np.zeros((sys_size,sys_size))

			# t = np.linspace(0,500,1000)
			# Loop over outputs and inputs for step response
			for outputs in range(0,sys_size):
				for inputs in range(0,sys_size):
					
					# Perform step experiment with variable time size, 0 as zero condition and the input,output
					y, t = cn.step(G,None,.0,inputs,outputs)
					u = np.ones_like(t)


					# Get the parameter
					K[outputs][inputs], T[outputs][inputs], L[outputs][inputs] = alg.Integral_Identification(y,u,t)
					
					# Delete
					#del y,u,t
			print(K)
			# Controller
			KY, KR = alg.Control_Decentral(K,T,L)
			
			# Make a Controller
			for outputs in range(0,sys_size):
				for inputs in range(0,sys_size):
					if (outputs == 0) and (inputs == 0):
						if (KY[outputs][inputs][0]  == 0) and (KY[outputs][inputs][1]==0):
							PIY = cn.ss(0,0,0,0)
							PIR = cn.ss(0,0,0,0)
							
						else:
							piy = cn.tf(KY[outputs][inputs][0],1)+cn.tf(KY[outputs][inputs][1],[1,0])
							pir = cn.tf(KR[outputs][inputs][0],1)+cn.tf(KR[outputs][inputs][1],[1,0])
							
							PIY = cn.tf2ss(piy)
							PIR = cn.tf2ss(pir)
					else:
						if (KY[outputs][inputs][0]  == 0) and (KY[outputs][inputs][1]==0):
							piy = cn.ss(0,0,0,0)
							pir = cn.ss(0,0,0,0)
							
						else:
							piy = cn.tf2ss(cn.tf(KY[outputs][inputs][0],1)+cn.tf(KY[outputs][inputs][1],[1,0]))
							pir = cn.tf2ss(cn.tf(KR[outputs][inputs][0],1)+cn.tf(KR[outputs][inputs][1],[1,0]))
							
						PIY = cn.bdalg.append(PIY,piy)
						PIR = cn.bdalg.append(PIR,pir)

			# Delete the old system
			#del G, y, u, t, num, den, den1, den2