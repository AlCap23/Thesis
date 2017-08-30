'''

Use this script from terminal / console with 
./python sisostudy.py --file_storage=my_runs

Will create an output with all the necessary information

TODO:
- Set Point Weight!
- Dynamisches Verhalten bewerten -> Ähnlichkeiten? Kann gut entkoppelt werden?
- Nur PT1 und Zeitkonstante nur zwischen 50...100 -> Ist das wirklich besser!?
- Wie bewertet man NANs?
- Setzte K iterativ ( for loop ) soweit hoch, bis system instabil! Bei 2x2
- Hierbei main diagonal = 1,1 , minor diagonal = 0.1...0.9
- ggf auch Zeitkonstanten vorgeben

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
	filename = 'TITO_290817_AST.csv'
	# Create the system numerator
	N = [[[1],[0.3]],[[0.2],[1]]]
	# Create the system denominator -> Several
	D = [[100,70],[60,90]]
	# Method 0 = RGA, 1= Aström, 2 = Modified Aström
	methods = 1

# Experimental Study
@ex.automain
def experiment(N,D,methods,filename):
	# Define the system
	num = N
	den = np.zeros((2,2,2))
	for outputs in range(0,2):
		for inputs in range(0,2):
			den[outputs][inputs][-1] = 1
			den[outputs][inputs][0] = D[outputs][inputs]
	den = den.tolist()
	
	# Define the system
	G = cn.tf2ss(cn.tf(num,den))
	
	# Preallocate the Parameter
	K = np.zeros((2,2))
	T = np.zeros((2,2))
	L = np.zeros((2,2))

	# Identify the system
	for outputs in range(0,2):
		for inputs in range(0,2):
			# Perform step experiment with variable time size, 0 as zero condition and the input,output
			y, t = cn.step(G,None,.0,inputs,outputs)
			u = np.ones_like(t)
			# Get the parameter
			K[outputs][inputs], T[outputs][inputs], L[outputs][inputs] = alg.Integral_Identification(y,u,t)
	
	# Get the controller
	# Check for methods and define controller

	if methods == 0:
		# Design decentralised controller
		KY,KR = alg.Control_Decentral(K,T,L)
	elif methods == 1:
		# Design Astrom controller with maximum decoupling = 0.1
		H = np.diag(0.1*np.ones(2))
		KY, KR = alg.Control_Astrom(K,T,L,H)
	elif methods == 2:
		# Design decoupled controller with maximum decoupling = 0.1
		H = np.diag(0.1*np.ones(2))
		KY, KR = alg.Control_Decoupled(K,T,L,H)
		
	# Make a Controller
	# Define PIY / PIR

	# Numerator Preallocate for all coefficients
	num_y = np.zeros((2,2,2))
	num_r = np.zeros((2,2,2))
	# Numerator Preallocate for all coefficients
	den = np.zeros((2,2,2))

	for outputs in range(0,2):
		for inputs in range(0,2):
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
		
	# Explore Sensitivity
		
	# Make unit feedback
	AS = np.zeros((2,2))
	AS = AS.tolist()
	DS = np.eye(2)
	DS = DS.tolist()
	G_One = cn.ss(AS,AS,AS,DS)

	# Sensitivity Tranfer Function
	S = cn.ss2tf(cn.matlab.minreal(cn.feedback(G_One, PIY*G)))
		
	# Scan for sensitivity until w_c = 1/T_min approx 1
	w = np.linspace(0,100,10000)
	# Store the maximum sensitivity and minimum 
	sigmax_s = np.zeros_like(w)
	sigmin_s = np.zeros_like(w)
	# Store the maximum gain at -180 deg

	# Get the magnitude, phase and response
	mag, phase, omega = cn.freqresp(S,w)
	# Find Maximum singular value
	for frequency in range(0,len(w)):
		# Singular Value decomposition
		u, s, v = np.linalg.svd(mag[:,:,frequency])
		# Store the singular values
		sigmax_s[frequency] = np.max(s)
		sigmin_s[frequency] = np.min(s)


	# Close the loop with and without setpoint

	# Store the maximum sensitivity and minimum 
	sigmax_c = np.zeros_like(w)
	sigmin_c = np.zeros_like(w)
	sigmax_b = np.zeros_like(w)
	sigmin_b = np.zeros_like(w)
	
	for setpoint in range(0,2):
		# Active setpoint , can be zero
		if setpoint == 0:
			CL = cn.ss2tf(cn.matlab.minreal(cn.feedback(G,PIY)*PIR))
			# Get the magnitude, phase and response
			mag, phase, omega = cn.freqresp(CL,w)
			# Find Maximum singular value
			for frequency in range(0,len(w)):
				# Singular Value decomposition
				u, s, v = np.linalg.svd(mag[:,:,frequency])
				# Store the singular values
				sigmax_b[frequency] = np.max(s)
				sigmin_b[frequency] = np.min(s)
		else:
			CL = cn.ss2tf(cn.matlab.minreal(cn.feedback(G*PIY,G_One)))
			# Get the magnitude, phase and response
			mag, phase, omega = cn.freqresp(CL,w)
			# Find Maximum singular value
			for frequency in range(0,len(w)):
				# Singular Value decomposition
				u, s, v = np.linalg.svd(mag[:,:,frequency])
				# Store the singular values
				sigmax_c[frequency] = np.max(s)
				sigmin_c[frequency] = np.min(s)
	# Create a DataFrame
	R = pd.DataFrame({'omega':w,'sigmax_s':sigmax_s, 'sigmin_s':sigmin_s, 'sigmax_b':sigmax_b, 'sigmin_b':sigmin_b, 'sigmax_c':sigmax_c, 'sigmin_c':sigmin_c})
	# Save DataFrame
	R.to_csv(filename, sep=';')