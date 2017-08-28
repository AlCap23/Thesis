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
	filename = 'MIMO_TEST_5.csv'
	# Sample size per system order
	sample_size = 1
	# Size, assuming quadratic system
	sys_size = 2
	# Maximum System Order (-1)
	max_order = 10
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
	columns = ['Sample No.', 'Order', 'sig_max_cl_D', 'sig_min_cl_D', 'sig_max_s_D', 'sig_min_s_D', 'sig_max_cl_A', 'sig_min_cl_A', 'sig_max_s_A', 'sig_min_s_A', 'sig_max_cl_M', 'sig_min_cl_M', 'sig_max_s_M', 'sig_min_s_M']
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
			
			# Save for reuse
			AS = np.zeros((sys_size,sys_size))
			AS = AS.tolist()
			DS = np.eye(sys_size)
			DS = DS.tolist()
			G_One = cn.ss(AS,AS,AS,DS)
			
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
			G = cn.tf2ss(cn.tf(num,den1))
			

			# Preallocate the Parameter
			K = np.zeros((sys_size,sys_size))
			T = np.zeros((sys_size,sys_size))
			L = np.zeros((sys_size,sys_size))

			# Loop over outputs and inputs for step response
			for outputs in range(0,sys_size):
				for inputs in range(0,sys_size):
					
					# Perform step experiment with variable time size, 0 as zero condition and the input,output
					y, t = cn.step(G,None,.0,inputs,outputs)
					u = np.ones_like(t)


					# Get the parameter
					K[outputs][inputs], T[outputs][inputs], L[outputs][inputs] = alg.Integral_Identification(y,u,t)
					
					# Delete
			
			# Preallocate the needed singular values
			# Preallocate the frequency
			w = np.logspace(-5,5,1000)
			# Singular values 
			sigmax = np.zeros((2,3,len(w)))
			sigmin = np.zeros((2,3,len(w)))
			# Iterate over the methods, 0 = Decentralized Control, 1= Astrom , 2 = Decoupling
			for methods in range(0,3):

				# Check method
				if methods == 0:
					# Design decentralised controller
					KY,KR = alg.Control_Decentral(K,T,L)
				elif methods == 1:
					# Design Astrom controller with maximum decoupling = 0.1
					H = np.diag(0.1*np.ones(sys_size))
					KY, KR = alg.Control_Astrom(K,T,L,H)
				elif methods == 2:
					# Design decoupled controller with maximum decoupling = 0.1
					H = np.diag(0.1*np.ones(sys_size))
					KY, KR = alg.Control_Decoupled(K,T,L,H)
			
				# Make a Controller

				# Define PIY / PIR

				# Numerator Preallocate for all coefficients
				num_y = np.zeros((sys_size,sys_size,2))
				num_r = np.zeros((sys_size,sys_size,2))
				# Numerator Preallocate for all coefficients
				den = np.zeros((sys_size,sys_size,2))

				for outputs in range(0,sys_size):
					for inputs in range(0,sys_size):
						# Set denominator to [1, 0] for striclty integral
						den[outputs][inputs][-2] = 1
						# Set the numerator to [kp, ki] 
						num_y[outputs][inputs][:] = [KY[outputs][inputs][0],KY[outputs][inputs][1]]
						num_r[outputs][inputs][:] = [KR[outputs][inputs][0],KR[outputs][inputs][1]]	

				# Make a list
				num_y = num_y.tolist()
				num_r = num_r.tolist()
				den = den.tolist()	

				# Make a system
				PIY = cn.tf2ss(cn.tf(num_y,den))
				PIR = cn.tf2ss(cn.tf(num_r,den))
				#print(PIY)	

				# Close the loop
				CL = cn.ss2tf(cn.feedback(G,PIY)*PIR)
				# Sensitivity
				S = cn.ss2tf(cn.feedback(G_One,PIY*G))	

				# Make frequency response
				mag1, phase1, omega = cn.freqresp(CL,w)
				mag2, phase2,omega = cn.freqresp(S,w)	

				for frequency in range(0,len(w)):
					# Closed loop characteristic
					u,sv,d = np.linalg.svd(mag1[:,:,frequency])
					sigmax[0,methods,frequency] = np.max(sv)
					sigmin[0,methods,frequency] = np.min(sv)
					# Sensitivity characteristic
					u,sv,d = np.linalg.svd(mag2[:,:,frequency])
					sigmax[1,methods,frequency] = np.max(sv)
					sigmin[1,methods,frequency] = np.min(sv)	


			
			# Columns	
			#['Sample No.', 'Order', 'sig_max_cl_D', 'sig_min_cl_D', 'sig_max_s_D', 'sig_min_s_D', 'sig_max_cl_A', 'sig_min_cl_A', 'sig_max_s_A', 'sig_min_s_A', 'sig_max_cl_M', 'sig_min_cl_M', 'sig_max_s_M', 'sig_min_s_M']
			# Append Data
			#if order == 1:
			R.loc[sys_no-1] = [sys_no, order,np.max(sigmax[0,0,:]), np.max(sigmin[0,0,:]),np.max(sigmax[1,0,:]), np.max(sigmin[1,0,:]),np.max(sigmax[0,1,:]), np.max(sigmin[0,1,:]),np.max(sigmax[1,1,:]), np.max(sigmin[1,1,:]),np.max(sigmax[0,2,:]), np.max(sigmin[0,2,:]),np.max(sigmax[1,2,:]), np.max(sigmin[1,2,:])]
			
			# Show percentage
			per = sys_no/((max_order-1)*sample_size)
			sys.stdout.write("\r %f" %per)
			sys.stdout.flush()

		R.to_csv(filename, sep=';')