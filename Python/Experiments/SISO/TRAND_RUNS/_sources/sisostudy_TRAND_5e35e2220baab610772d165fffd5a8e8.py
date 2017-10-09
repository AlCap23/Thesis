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
import matplotlib.pyplot as plt

import sys


# Define an experiment
from sacred import Experiment
ex = Experiment()

# Configuration
@ex.config
def experimental_setup():
	# Set up the Experiment and define the range of system gain, lag and delay as well as filename etc
	filename = 'SISO_06102017_TRAND.csv'
	# Sample size per system order
	sample_size = 1000
	# Maximum System Order (-1)
	max_order = 10
	# System noise in percent of gain
	noise_limit = 0
	# Gain Limits
	gain_limits = [-10,10]
	# Lag Limits 
	lag_limits = [1,20]
	# Delay Limits -> If small no delay
	delay_limits = [0, 20]
	# Create the system numerator
	N = np.random.uniform(gain_limits[0],gain_limits[1],(max_order,sample_size))
	# Create the system denominator -> Several
	D = np.random.uniform(lag_limits[0],lag_limits[1],(max_order,sample_size,max_order))
	# Create the system delay
	L = np.random.uniform(delay_limits[0], delay_limits[1],(max_order,sample_size))
	# Create an array for the results: sys_no, order, K,T, k,t,l, ms_real,ms_ideal, 4x t_rise, mp, t_settle, yss 
	columns = ['Sample No.', 'Order', 'K', 'TSum', 'L','KM', 'TM', 'LM', 'MS Real', 'MS Ideal', 'MT Real', 'MT Ideal']
	R = pd.DataFrame(columns=columns)

# Experimental Study
@ex.automain
def experiment(N,D,L,R,noise_limit,sample_size,max_order,filename, columns):
	# Define system no
	sys_no = 0
	# Define an outer loop over system order
	for order in range(1,max_order):
		# Define an inner loop over samples
		for sample in range(0,sample_size):
			# Current System
			sys_no += 1
			# Get the denominator
			den = [D[order][sample][order],1]
			# Get the gain
			num = N[order][sample]
			# Get the delay
			l = L[order][sample]

			# Define a Transfer Function with pole multiplicity
			G = cn.tf([1],den)
			# Add gain and rise system order
			if order > 1:
				for i in range(2,order):
					G = G*cn.tf([1],[D[order][sample][i],1])
			# Add gain	
			G = num*G
			# Add delay with pade approximation of order 10
			num, den = cn.pade(l,10)
			G = G*cn.tf(num,den)

			# Step response
			t = np.linspace(0,1000,10000)
			y,t = cn.step(G,t)
			
			#plt.clf()
			#plt.plot(t,y)
			#plt.show()

			# Add noise 
			if noise_limit > 1e-3:
				y = y + np.random.normal(0,noise_limit*N[order][sample],y.size)
			u = np.ones_like(t)

			# Identify the system
			km,tm,lm = alg.Integral_Identification(y,u,t)

			# Make a model of the system
			num,den = cn.pade(lm,10)
			GM = cn.tf([km],[tm,1])*cn.tf(num,den)

			# Tune AMIGO controller
			params, b = alg.AMIGO_Tune(km,tm,lm)
			# Define a Controller with setpoint
			ky = cn.tf([params[0]],[1])+cn.tf([params[1]],[1,0])
			kr = cn.tf([b*params[0]],[1])+cn.tf([params[1]],[1,0])
		
			# REAL SYSTEM
			# Real system sensitivity
			real_sens = 1/(1+G*ky)
			# Real system complementary sensitivity
			real_comp = real_sens*G*kr

			# IDENTIFIED SYSTEM
			# Identified system sensitivity
			iden_sens = 1/(1+GM*ky)
			# Identified system complementary sensitivity
			iden_comp = iden_sens*GM*kr
			
			# Step response
			#y_rclsw,t_rclsw = cn.step(real_clsw)
			#y_rcl,t_rcl = cn.step(real_cl)
			#y_iclsw,t_iclsw = cn.step(iden_clsw)
			#y_icl, t_icl = cn.step(iden_cl)

			# Compute the gain
			# Define Frequency range
			omega = np.logspace(-5,5,1000)
			gain, phase, omega = cn.bode(real_sens,omega)
			MS_Real = np.max(gain)
			gain,phase,omega = cn.bode(real_comp,omega)
			MT_Real = np.max(gain)
			gain, phase, omega = cn.bode(iden_sens,omega)
			MS_Iden = np.max(gain)
			gain,phase,omega = cn.bode(iden_comp,omega)
			MT_Iden = np.max(gain)

			# Get the Step Information
			#Tr_RSW, Mp_RSW, Ts_RSW, Ys_RSW = alg.Step_Info(y_rclsw,t_rclsw)
			#Tr_R, Mp_R, Ts_R, Ys_R = alg.Step_Info(y_rcl,t_rcl)
			#Tr_ISW, Mp_ISW, Ts_ISW, Ys_ISW = alg.Step_Info(y_iclsw,t_iclsw)
			#Tr_I, Mp_I, Ts_I, Ys_I = alg.Step_Info(y_icl,t_icl)

			
			# Append Data
			#if order == 1:
			R.loc[sys_no-1] = [sys_no, order, N[order][sample], np.sum(D[order][sample][:order]),L[order][sample], km, tm, lm, MS_Real, MS_Iden, MT_Real, MT_Iden]
			#else:
			#	R.loc[sys_no-1] = [sys_no, order, N[order][sample], np.sum(D[order][sample][0:order-1]), km, tm, lm, MS_Real, MS_Iden, Tr_RSW, Mp_RSW, Ts_RSW, Ys_RSW, Tr_R, Mp_R, Ts_R, Ys_R, Tr_ISW, Mp_ISW, Ts_ISW, Ys_ISW, Tr_I, Mp_I, Ts_I, Ys_I]
			per = float(sample)/float(sample_size)*100
			sys.stdout.write("\r %.2f" %per)
			sys.stdout.flush()

		R.to_csv(filename, sep=';')
			


