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
# Import linear regression model
from scipy import stats
# Import the Algorithms
import sys
sys.path.append('../../')
import Algorithms as alg
# Import the MOBA Simulator
import MoBASimulator as mb 
# Plotting
import pylab as p
# Define an experiment
from sacred import Experiment

###########################################################
########################## FUNCTIONS ######################
###########################################################

# Function for the transfer function matrix of a system given a steady state rep 
# and a frequency for a 2 Output 2 Input system

def tf_system(ss, omega):
	# Get the matrices
	A = ss['A']
	B = ss['B']
	C = ss['C']
	D = ss['D']
	# Make a I matrix ( depends on the states)
	I = np.eye(A.shape[0])
	# The Transfer Function
	G = np.dot(np.dot(C,np.linalg.inv(omega*1j*I-A)),B)+D

	return G

# Compute a controller for a given KY, B, D
def compute_pi(KY, B, D):
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

	return KPR, KIR, KPY, KIY

# Compute the sensitivity function of a closed loop
# Takes system, controller and frequency
def compute_sensitivity(ss,KY,B,D,omega):
	# Compute the transfer function matrix
	G = tf_system(ss, omega)
	# Compute the controller
	KPR, KIR, KPY, KIY = compute_pi(KY,B,D)
	# Compute the sensitivity
	S = np.linalg.inv(np.eye(2,2)-np.dot(G,np.add(KPY,1/(omega*1j)*KIY)))
	return S

# Compute complementary sensitivity of a closed loop
# Takes system, controller and frequency
def compute_complementarysensitivity(ss, KY, B, D, omega):
	# Compute the transfer function matrix
	G = tf_system(ss, omega)
	# Compute the controller
	KPR, KIR, KPY, KIY = compute_pi(KY,B,D)
	# Compute the sensitivitiy
	S = compute_sensitivity(ss, KY, B, D, omega)
	# Compute the complementary sensitivity
	T = np.dot(S,np.dot(G, np.add(KPR,1/(omega*1j)*KIR) ))
	return T

###########################################################
########################## MAIN PROGRAM ###################
###########################################################


# Define a Sacred Experiment
ex = Experiment()

###########################################################
########################## CONFIG #########################
###########################################################

@ex.config 
def experimental_setup():
	# Filename to store in
	filename  = 'TITOStudy_external_15092017_1.csv'
	# Overall sample_size 
	sample_size = 9
	# Max degree
	max_deg = 9
	# Gain Limits
	gain_limits = [-10., 10.0]
	# Lag Limits
	lag_limits = [80,180]
	# Delay Limits, if small approx. no delay
	delay_limits = [1,10]
	# Step size for simulate
	dt = 0.01
	# Final time for simulation
	t_sim = 1500
	# Maximum Interaction
	H = 0.5*np.eye(2,2)
	# Frequency parameter (as dB)
	wmin = -5
	wmax = 3
	dw = 10000
	# Special frequencies
	w_special = np.array([0.1, 0.5, 1.0])
	# Store the results
	# System Order, Maximum Sensitivity, corresponding Frequency, MS_w0.1, MS_w0.5, MS_w1, Grad_MS(w0.1...1)
	columns = ['Degree','MS_RGA','w_MS_RGA','Grad_RGA','MS_A', 'w_MS_A','Grad_A', 'MS_D','w_MS_D', 'Grad_D']
	# Add the special frequencies
	for freq in range(0, w_special.shape[0]):
		columns.append('w_'+str(w_special[freq])+'_RGA')
		columns.append('w_'+str(w_special[freq])+'_A')
		columns.append('w_'+str(w_special[freq])+'_D')
	# Make empty data frame with zeros
	R = pd.DataFrame(data = np.zeros((sample_size, len(columns))), columns = columns)

	###########################################################
	################## CREATE VARIABLES #######################
	###########################################################

	# Create the gain
	k = np.random.uniform(gain_limits[0], gain_limits[1],(sample_size,2,2))
	num = np.zeros_like(k)

	# Delay
	l = np.random.uniform(delay_limits[0], delay_limits[1], (sample_size,2,2))

	# Create random time constants
	t = np.random.uniform(lag_limits[0],lag_limits[1],(sample_size,2,2))
	den = np.zeros((sample_size, 2, 2, max_deg+1))
	
	# Loop over the samples and estimate even distribution over degree
	for samples in range(0, sample_size):
		# Compute current order, from 1 to ...
		degree = int(1.0*samples/sample_size * max_deg) + 1
		# Loop over outputs
		for outputs in range(0,2):
			# Loop over inputs
			for inputs in range(0,2):
				# Compute the distances between the random time constants
				# Sort until current degree
				dist = float(t[samples, outputs, inputs]/degree) * np.ones(degree)
				# Insert a zero for the first distance
				#dist = np.insert(dist, [0], 0.0)
				# Calculate the distance 
				#dist = np.ediff1d(dist)
				# Calculate a stable polynomial, which highest coefficient is normed!!!
				den[samples, outputs, inputs, :(degree+1)] = np.polynomial.polynomial.polyfromroots(-1./dist)
				# Hence, normalize the gain with the highest coefficient
				num[samples, outputs, inputs] = k[samples, outputs, inputs] * den[samples, outputs, inputs, 0]

###########################################################
################## EXPERIMENT #############################
###########################################################	

@ex.automain
def experiment(num, den, l, R, filename, sample_size, max_deg, dt, t_sim, H, wmin, wmax, dw, w_special):
	# Loop over the samples, compute order like earlier

	###########################################################
	####################### INITIAL MODEL #####################
	###########################################################

	# Open Simulator
	sim = mb.Simulator()
	# Show the log window
	sim.showLogWindow()

	###########################################################
	####################### SAMPLE LOOP #######################
	###########################################################
	
	# Set initial degree to zero
	degree = 0

	for samples in range(0, sample_size):

		# Calculate the current degree
		c_deg = int(1.0 * samples/sample_size * max_deg) +1
		# Check if degree has changed
		if degree < c_deg:
			# Change degree
			degree = c_deg
			# Clear Simulator
			sim.clear()
			# Load new model
			sim.loadModel("C:/Users/juliu/Documents/Thesis/Modelica/FMU/2_2_n"+str(degree)+"/Masterthesis_Models_mimo_0processmodel.fmu")
			sim.setOperationMode('FMU for ModelExchange')

		# Preallocat identification parameter
		K = np.zeros((2,2))
		T = np.zeros((2,2))
		L = np.zeros((2,2))

		# Reload the model
		sim.reloadModel()

	###########################################################
	####################### MODEL SETUP #######################
	###########################################################

		# Create a parameter list
		params = {}

		# Loop over the systems outputs
		for outputs in range(0,2):
			# Loop over the systems inputs
			for inputs in range(0,2):
				# Set system gain
				params.update({"fmu.num["+str(outputs+1)+","+str(inputs+1)+",1]": num[samples][outputs][inputs]})
				# Set system delay
				params.update({"fmu.delay["+str(outputs+1)+","+str(inputs+1)+"]": l.item(samples,outputs,inputs)})
				# Loop over denominator coefficients
				for order in range(0, degree+1):
					params.update({"fmu.den["+str(outputs+1)+","+str(inputs+1)+","+str(degree-order+1)+"]": den[samples][outputs][inputs][(order)]})

		# Set the parameter
		sim.set(params)
		# Show the Parameter
		#sim.showParameterDialog()
		# Store the state space rep for later use
		ss = sim.analyser_getStateSpaceForm()

	###########################################################
	####################### IDENTIFICATION ####################
	###########################################################

		# Setup first experiment Input 1 -> Output 1 and Output 2
		sim.set({"fmu.u[1]": 1,"fmu.u[2]": 0})
		# Simulation of the experiment
		res = sim.simulate(dt, t_sim)
		# Get the needed signals
		y = res["fmu.y[1]"]
		y2 = res["fmu.y[2]"]
		u = res["fmu.u[1]"]
		time = res["time"]

		# Plot the system
		p.plot(time,y)
		p.plot(time,y2)
		p.show()

		# Get TF from Input 1 to Output 1
		K[0][0],T[0][0],L[0][0]=alg.Integral_Identification(y,u,time)
		# Get TF from Input 1 to Output 2
		K[1][0],T[1][0],L[1][0]=alg.Integral_Identification(y2,u,time)

		# Setup second experiment Input 2 -> Output 1 and Output 2
		# Reset the model state
		sim.resetModelState()
		# Input Parameter
		sim.set({"fmu.u[1]": 0,"fmu.u[2]": 1})
		# Simulation of the experiment
		res = sim.simulate(dt, t_sim)
		# Get the needed signals
		y = res["fmu.y[1]"]
		y2 = res["fmu.y[2]"]
		u = res["fmu.u[2]"]
		time = res["time"]

		# Get TF from Input 2 to Output 1
		K[0][1],T[0][1],L[0][1] = alg.Integral_Identification(y,u,time)
		# Get TF from Input 2 to Output 2
		K[1][1],T[1][1],L[1][1] = alg.Integral_Identification(y2,u,time)
		
		# Print the System Parameter	
		# print(K,T,L)

	###########################################################
	####################### CONTROLLER DESIGN #################
	###########################################################
		
		# Loop over the three methods
		for methods in range(0,3):
			if methods == 0:
				KY,B,D = alg.Control_Decentral(K,T,L)
			elif methods == 1:
				KY,B,D = alg.Control_Astrom(K,T,L,H)
			else:
				KY,B,D = alg.Control_Decoupled(K,T,L,H)

	###########################################################
	####################### EVALUATION ########################
	###########################################################
			# Create a frequency range
			omega = np.logspace(wmin, wmax, dw)
			# Store the singular values
			sv = np.zeros((2,omega.shape[0]))
			# Loop over the frequency
			for freq in range(0, omega.shape[0]):
				# Evaluate the sensitivity at given frequency
				S = compute_sensitivity(ss, KY,B,D, omega[freq])
				u, sv[:, freq], w = np.linalg.svd(np.abs(S))
			# Clear variables
			del u,w
			# Find the maximum of the sensitivity
			ms = np.max(sv)
			# Get the corresponding frequency
			omega_ms = omega[np.argmax(sv)]

			# Print the sensitivity
			#p.loglog(omega, sv[0,:])
			#p.loglog(omega, sv[1,:])
			#p.show()

			# Compute the gradient of the maximal singular values 
			# Compute the maximum singular value along all frequency
			sv_max = np.max(sv, axis=0)
			# Compute the slope via linear regression
			slope, intercept, r_value, p_value, std_err = stats.linregress(omega[np.where(omega<=1.0)], sv_max[np.where(omega<=1.0)])
			# Clear variables
			del intercept, r_value, p_value, std_err
			# Evaluate at the special frequencies
			ms_s = []

			for freq in w_special:
				# Evaluate the sensitivity at given frequency
				S = compute_sensitivity(ss, KY,B,D, freq)
				u, v, w = np.linalg.svd(np.abs(S))
				ms_s.append(np.max(v))
			# Clear variables
			del u,v,w	

	###########################################################
	####################### STORE DATA ########################
	###########################################################

			# Store Degree
			R.set_value(samples, 'Degree', degree)

			if methods == 0:
				# Store the maximum sensitivity
				R.set_value(samples, 'MS_RGA', ms)
				# Store the correspondig frequency
				R.set_value(samples, 'w_MS_RGA', omega_ms)
				# Store the maximum singular value at the special frequencies
				for freq in range(0, w_special.shape[0]):
					R.set_value(samples, 'w_'+str(w_special[freq])+'_RGA', ms_s[0])
				# Store the gradient
				R.set_value(samples, 'Grad_RGA', slope)
			elif methods == 1:
				# Store the maximum sensitivity
				R.set_value(samples, 'MS_A', ms)
				# Store the correspondig frequency
				R.set_value(samples, 'w_MS_A', omega_ms)
				# Store the maximum singular value at the special frequencies
				for freq in range(0, w_special.shape[0]):
					R.set_value(samples, 'w_'+str(w_special[freq])+'_A', ms_s[0])
				# Store the gradient
				R.set_value(samples, 'Grad_A', slope)
			else: 
				# Store the maximum sensitivity
				R.set_value(samples, 'MS_D', ms)
				# Store the correspondig frequency
				R.set_value(samples, 'w_MS_D', omega_ms)
				# Store the maximum singular value at the special frequencies
				for freq in range(0, w_special.shape[0]):
					R.set_value(samples, 'w_'+str(w_special[freq])+'_D', ms_s[0])
				# Store the gradient
				R.set_value(samples, 'Grad_D', slope)
		# Store after every sample
		R.to_csv(filename, sep=";")
