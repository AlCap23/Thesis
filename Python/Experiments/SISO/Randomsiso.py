'''
READ ME

Use this script from terminal / console with 
./python SISOExperiment.py --file_storage=my_runs

Will create an output with all the necessary information
'''

# Import the pacakges
# Numpy for numerical methods
import numpy as np 
# Python Control for SISO creation etc.
import control as cn

# Import the Algorithms
import sys
sys.path.append('../../')
#alg = importlib.machinery.SourceFileLoader('Algorithms','../../').load_module()
import Algorithms as alg



# Define an experiment
from sacred import Experiment
RandomSys = Experiment()


# Add the configuration in here -> all needed variables
@RandomSys.config
def my_config():
	# Filename to store information
	filename = 'test.csv'
	# Current Order
	order = 1
	# Gain Limit
	min_gain = 0.1
	max_gain = 10
	# Lag Limit 
	min_lag = 1
	max_lag = 10
	# Create the numerator
	num = np.random.uniform(min_gain,max_gain)
	# Create the denominator 
	den = np.array([np.random.uniform(min_lag, max_lag),1])**order	


# Define the main Experiment
# This is where the magic happens
@RandomSys.automain
def System_Study(order,num,den,filename):
	# Create a system
		# Create a linear transfer function
	G = cn.tf(num,den.tolist())
	
	# Step response
	y,t = cn.step(G)
	u = np.ones_like(y)
			
	# Identify the system with FOTD model
	k,t,l = alg.Integral_Identification(y,u,t)
	padnum,padden = cn.pade(l,10)
	GM = cn.tf([k],[t,1])*cn.tf(padnum,padden)

	# Tune a controller with AMIGO - PI Controller
	params, b = alg.AMIGO_Tune(k,t,l)
	# Define a Controller with setpoint
	ky = cn.tf([params[0]],[1])+cn.tf([params[1]],[1,0])
	kr = cn.tf([b*params[0]],[1])+cn.tf([params[1]],[1,0])
		
	# REAL SYSTEM
	# Real system closed loop, setpoint weight
	real_clsw = cn.feedback(G,ky)*kr
	# Real system closed loop, without setpoint weight
	real_cl = cn.feedback(G*ky,1)
	# Real system sensitivity
	real_sens = 1/(1+G*ky)

	# IDENTIFIED SYSTEM
	# Identified system closed loop, setpoint weight
	iden_clsw = cn.feedback(GM,ky)*kr
	# Identified system closed loop, without setpoint weight
	iden_cl = cn.feedback(GM*ky,1)
	# Identified system sensitivity
	iden_sens = 1/(1+GM*ky)
	
	# Compute the Values

	# Sensititvity
	# Frequency spectrum from 10^-5 to 10^5 in 10000 steps
	w = np.logspace(-5,5,10000)
	# Compute the magnitude over frequency
	mag,phase,freq = cn.matlab.bode(real_sens,w)
	# Compute the maximum of magnitude
	real_MS = np.amax(mag)
	# Repeat
	mag,phase,freq = cn.matlab.bode(iden_sens,w)
	# Compute the maximum of magnitude
	iden_MS = np.amax(mag)

	# Calculate the Rising Time

	# Calculate the Settling Time

	# Calculate the Overshoot / Undershoot

	# Save the parameter
	P = [num, den, k,t,l ,params, b,  real_MS, iden_MS]
	return(num, den, k,t,l ,params, b,  real_MS, iden_MS)