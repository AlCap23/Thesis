"""
Python programm to study the robustness of TITO systems. 
Identitfies the system, computes the controller and analysis the controller using the state space - transfer function relation.
Computes the singular values.

Use this script from the terminal / console with
./python FILENAME.py --file_storage = FOLDERNAME
to store essential information with sacred
"""

# Import the packages
# Import numpy
import numpy as np 
# Import pandas
import pandas as pd
# Import the Algorithms
import sys
sys.path.append('../../')
import Algorithms as alg
# Import the MOBA Simulator
import MoBASimulator as mb 

# Define an experiment
from sacred import Experiment
ex = Experiment()

# Configuration
@ex.config 
def experimental_setup():
	# Filename to store in
	filename  = 'TITOStudy_15092017_2.csv'
	# Sample Size
	sample_size = 10
	# Max order
	max_order = 9
	# Gain Limits
	gain_limits = [2., 30.]
	# Pole Limits
	pole_limits = [-1.0, -.1]
	# Zero Limits 
	zero_limits = [-1.0, 1.0]
	# Delay Limits, if small approx. no delay
	delay_limits = [1e-10, 1e-3]
	# Store the results
	# Sample number, gain, sum time, model gain, model lag, model delay, max sensitivity and corresponding frequency, max comp- sensitivity and corresponding freq
	columns = ['Sample No', 'MS_RGA', 'Freq_MS_RGA', 'MCS_RGA', 'Freq_MCS_RGA','MS_A', 'Freq_MS_A', 'MCS_A', 'Freq_MCS_A','MS_D', 'Freq_MS_D', 'MCS_D', 'Freq_MCS_D']
	R = pd.DataFrame(columns = columns)

############################################################################
	# Create the numerator and denumerator

	# Gains
	k = np.random.uniform(gain_limits[0], gain_limits[1],(2,2,sample_size))

	
	# Poles, 2 Outputs, 2 Inputs, polynomial degree 8, samples 
	p = np.random.uniform(pole_limits[0],pole_limits[1],(2,2,8, sample_size))
	# Denominator coefficients
	den = np.zeros((2,2,9,sample_size))

	# Zeros are not implemented
	z = np.zeros((2,2,8, sample_size))
	# Set the last entry to ones
	z[:][:][0][:] = np.ones_like(z[:][:][0][:])
	num = 1e-10*np.ones((2,2,9,sample_size))
	num[:][:][0][:] = np.ones_like(num[:][:][0][:])

	# Calculate the numerator and denominator
	for sample in range(0,sample_size):
		for outputs in range(0,2):
			for inputs in range(0,2):
				den[outputs,inputs,:,sample] = np.polynomial.polynomial.polyfromroots(p[outputs,inputs,:,sample])
				#num[outputs,inputs,:,sample] = np.polynomial.polynomial.polyfromroots(z[outputs,inputs,:,sample])

	# Delay
	l = np.random.uniform(delay_limits[0], delay_limits[1], (2,2,sample_size))
	

############################################################################

# Experimental study
@ex.automain
def experiment(k,l,num,den,R,filename, sample_size, max_order):
	# Define the sample number
	sample_no = 0
	# Load the system
	sim = mb.Simulator()
	sim.clear()
	sim.loadModel("C:/Users/juliu/Documents/Thesis/Modelica/FMU/2_2_n9/Masterthesis_Models_mimo_0processmodel.fmu")
	sim.setOperationMode('FMU for ModelExchange')
	# Show the log window
	sim.showLogWindow()
	# Loop over samples
	for sample in range(0,sample_size):
		sample_no +=1
		# Create empty parameter Matrices
		K = np.zeros((2,2))
		T = np.zeros((2,2))
		L = np.zeros((2,2))
		# Reload the model
		sim.reloadModel()
		# Create a parameter list
		params = {}
		# Set the parameter
		# Loop over system size
		for outputs in range(1,3):
		    for inputs in range(1,3):
		        # Process Parameter 
		        # System Lag
		        for order in range(0,9):
		            params.update({"fmu.den["+str(outputs)+","+str(inputs)+","+str(order+1)+"]": den[outputs-1][inputs-1][order][sample]})
		            # System Gain
		            #params.update({"fmu.num["+str(outputs)+","+str(inputs)+","+str(order+1)+"]": k[outputs-1][inputs-1]})
		            #params.update({"fmu.num["+str(outputs)+","+str(inputs)+","+str(order+1)+"]": k.item(outputs-1,inputs-1,sample)*num[outputs-1][inputs-1][order][sample]})
		        # System Gain
		        params.update({"fmu.num["+str(outputs)+","+str(inputs)+",1]": k[outputs-1][inputs-1][sample]})
		        # System Delay
		        params.update({"fmu.delay["+str(outputs)+","+str(inputs)+"]": l.item(outputs-1,inputs-1,sample)})

		# Set the parameter values
		sim.set(params)
		#sim.showParameterDialog()
		# Store the state space representation
		ss = sim.analyser_getStateSpaceForm()

		# Identify the mothertruckin system
		# First run, Input 1 -> Output 1 & 2
		sim.set({"fmu.u[1]": 1,"fmu.u[2]": 0})
		# Set timestep = 1e-2, endtime = 100
		res=sim.simulate(0.01, 1000)

		# Get the signals
		y = res["fmu.y[1]"]
		y2 = res["fmu.y[2]"]
		u = res["fmu.u[1]"]
		time = res["time"]
		
		# Get TF from Input 1 to Output 1
		K[0][0],T[0][0],L[0][0]=alg.Integral_Identification(y,u,time)
		# Get TF from Input 1 to Output 2
		K[1][0],T[1][0],L[1][0]=alg.Integral_Identification(y2,u,time)


		# Second run, Input 2 -> Output 1 & 2
		# Reload the Model to set everything to zero
		sim.resetModelState()
		sim.set({"fmu.u[1]":0, "fmu.u[2]":1})
		# Set timestep = 1e-2, endtime = 100
		res=sim.simulate(0.01, 1000)

		# Get the signals
		y = res["fmu.y[1]"]
		y2 = res["fmu.y[2]"]
		u = res["fmu.u[2]"]
		time = res["time"]

		# Get TF from Input 2 to Output 1
		K[0][1],T[0][1],L[0][1] = alg.Integral_Identification(y,u,time)
		# Get TF from Input 2 to Output 2
		K[1][1],T[1][1],L[1][1] = alg.Integral_Identification(y2,u,time)
		

		# Iterate over the controller Methods
		for methods in range(0,3):
			if methods == 0:
				KY,B,D = alg.Control_Decentral(K,T,L)
			elif methods == 1:
				KY,B,D = alg.Control_Astrom(K,T,L, 0.5*np.eye(2,2))
			else:
				KY,B,D = alg.Control_Decoupled(K,T,L, 0.5*np.eye(2,2))
			
			# Compute the controller
			# Make KPR,KPY, KIR and KIY
			KPR = np.zeros((2,2))
			KPY = np.zeros((2,2))
			KIR = np.zeros((2,2))
			KIY = np.zeros((2,2))
			# Fill with values
			for outputs in range(0,2):
			    for inputs in range(0,2):
			        # Proportional Controller
			        KPY[outputs,inputs] = KY[outputs,inputs,0]
			        # Intergral Controller
			        KIY[outputs,inputs] = KY[outputs,inputs,1]

			# Implement Set-point Weight
			KPR = np.dot(B,KPY)
			KIR = KIY

			# Compute the sensitivities
			# Store the Matrices
			A = ss['A']
			B = ss['B']
			C = ss['C']
			DSys = ss['D'] # Name different since decoupler D
			I = np.eye(A.shape[0])
			# Array for singular values
			sv = np.zeros((2,1000))
			counter = 0

			# Frequency Spectrum
			omega = np.logspace(-10,3,10000)
			# Singular Value Storage for sensitivity
			sv_s = np.zeros((2,omega.shape[0]))
			# Singular Value Storage for complementary sensitivity
			sv_t = np.zeros((2,omega.shape[0]))

			# Counter for storage
			counter = 0
			for freq in omega:
				# Transfer function gain
			    G = np.dot(np.dot(C,np.linalg.inv(freq*1j*I-A)),B)+DSys
			    # Sensitivity
			    S = np.linalg.inv(np.eye(2,2)-np.dot(G,np.add(KPY,1/(freq*1j)*KIY)))
			    u,sv_s[:,counter],w = np.linalg.svd(np.abs(S))
			    # Complementary sensitivity
			    CS = np.dot(S,np.dot(G, np.add(KPR,1/(freq*1j)*KIR) ))
			    u,sv_t[:,counter],w = np.linalg.svd(np.abs(T))
			    counter = counter +1

			# Get the maximum sensitivity
			max_ms =  np.max(sv_s)
			omega_ms = omega[np.argmax(sv_s)]

			# Get the maximum complementary sensitivity
			max_cms = np.max(sv_t)
			omega_cms = omega[np.argmax(sv_t)]
			
			if methods == 1:
				MS_RGA = max_ms
				Freq_MS_RGA = omega_ms
				MCS_RGA = max_cms
				Freq_MCS_RGA = omega_cms
			elif methods == 2:
				MS_A = max_ms
				Freq_MS_A = omega_ms
				MCS_A = max_cms
				Freq_MCS_A = omega_cms
			else:
				MS_D = max_ms
				Freq_MS_D = omega_ms
				MCS_D = max_cms
				Freq_MCS_D = omega_cms
		# Store the data
		R.loc[sample_no-1] = [sample_no, MS_RGA, Freq_MS_RGA, MCS_RGA, Freq_MCS_RGA, MS_A, Freq_MS_A, MCS_A, Freq_MCS_A, MS_D, Freq_MS_D, MCS_D, Freq_MCS_D]
		# Show percentage
		per = float(sample_no)/float(sample_size)*100.0
		sys.stdout.write("\r Current Status %f of 100\r" %per )
		sys.stdout.flush()
		# Store
		R.to_csv(filename, sep=";")

