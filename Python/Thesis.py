# Import packages
import control as cn 
import numpy as np 

def PI_11( tf, ):
	"""
	A function that returns the parameters of a PI controller designed
	on the paper "Design of PI Controllers based on Non-Convex Optimization"
	by Aström, Panagopoulous and Hägglund
	
	PI Controller Form is
	u(t) = k (b*r-y) + 1/(s*Ti) (r-y)

	Inputs
	tf - transfer function of the system

	Outputs
	Coeff - List of coefficients [k, Ti]
	"""

	print("Not implemented yet !")

	return;

def PI_FO(tf, omega, damping, forcing = 0):
	"""
	A function that returns the parameters of a PI controller designed
	on Pole Placement for a first order system

	PI Controller Form is
	u(t) = k (b*r-y) + 1/(s*Ti) (r-y)

	Inputs
	tf - transfer function of the system
	omega - critical frequency
	damping - Damping coefficient
	forcing - if TRUE the function simplifies the tf to be a FO system via 0.63 percent

	Outputs
	Coeff - List of coefficients [k, Ti]
	"""



	# Check if system is First Order by checking the size of the denominator
	if np.shape(cn.tfdata(tf)[1])[-1] ==2:
		# Get the coefficients - gain kp and time constant T1
		kp =cn.tfdata(tf)[0][-1][-1]
		T1 = cn.tfdata(tf)[1][0][0][0]
		# Calculate the coefficients
		k = (2*damping*omega*T1-1)/(kp)
		Ti = (2*damping*omega*T1-1)/(omega**2 *T1)
		return [k, Ti];

	elif forcing == 1:
		print("Forcing Model to fit First Order Model")
		print("Not implemented ! ")
		return;
	else:
		print("No first Order Model ! Either simplify model or turn on forcing!")
		return;



	return Coeff;

def RGA_PI( sys , output="S"):
	"""
	A function that creates a Controller for a multiple input multiple
	output ( MIMO ) system consisting of transfer functions

	Inputs: System ( as defined by the python control package)
	Outputs: Controller 
	- S in output: as a dynamical system in the python control package)
	- C in output: as Coefficients
	"""

	# Check the size of the system
	sys_size = np.shape(cn.tfdata(sys))
	#print(sys_size)

	# Compute the relative gain array
	RGA = np.multiply(cn.dcgain(sys),np.linalg.inv(cn.dcgain(sys)).T)

	# Find the maximum enty per output, so to speak the axis of the RGA
	sys_pairing = np.squeeze(np.argmax(RGA, axis=1).tolist())

	# Now make a controller for the tf in the optimal Input
	P = [[] for inputs in range(0,sys_size[1])]
	for outputs in range(0,sys_size[1]):
		# Call Control PID - still not written. Returns PI Controller Coefficients
		P[outputs] = [5.,7.]
	#print(P)
	#print(sys_pairing)
	# Check output
	if output is "C":
		return P, sys_pairing;
	else:
		# Make a Controller
		# Start by creating a zero array of same size as the system
		num = [[[0.0] for inputs in range(0,sys_size[2])] for outputs in range(0, sys_size[1])]
		den = [[[1.0] for inputs in range(0,sys_size[2])] for outputs in range(0, sys_size[1])]
		print(num, den)
		print(np.shape(num), np.shape(den))
		# Proportional Gain
		outputs = 0
		for inputs in sys_pairing:
				num[outputs][inputs] = [np.divide(P[outputs][0],P[outputs][1]),P[outputs][0]]
				outputs += 1
		# Integral Gain
		outputs = 0
		for inputs in sys_pairing:
			den[outputs][inputs] = [P[outputs][1], 0.]
			outputs += 1
		C = cn.tf(num,den)
		return C;

def Decoupling_PI_A( sys , output="S"):
	"""
	A function that creates a Controller for a multiple input multiple
	output ( MIMO ) system consisting of transfer functions.

	Function is based on the Paper "Design of decoupled PIUD controllers" by Aström et. al 

	Inputs: System ( as defined by the python control package)
	Outputs: Controller 
	- S in output: as a dynamical system in the python control package)
	- C in output: as Coefficients
	"""

	"""
	TODO

	Check loops, since we just iterate through the main Diagonal!
	SYSTEM HAS TO BE SQUARE!!!
	"""

	# Check the size of the system
	sys_size = np.shape(cn.tfdata(sys))
	sys_coeff = cn.tfdata(sys)
	#print(sys_size)

	# Compute the decoupling matrix D and make a transfer function
	D = np.linalg.inv(cn.dcgain(sys))
	D = np.asarray(D).reshape((sys_size[0],sys_size[1],1)).tolist()
	D = cn.tf(D,np.ones_like(D).tolist())


	# Compute the scaled system Q
	Q = G*D

	# Get the coefficients for the influence near steady state
	# Get the coefficients for the influence near steady state
	k = [ outputs for outputs in range(sys_size[1]*(sys_size[0]-1))]
	counter = 0
	for outputs in range(0, sys_size[1]):
    	for inputs in range(0, sys_size[2]):
        	print(outputs)
        	print(inputs)
        	if outputs != inputs:
            	print(sys_coeff[0][outputs][inputs][0])
            	print(sys_coeff[1][outputs][inputs][0])
            	k[counter] = sys_coeff[0][outputs][inputs][0] / sys_coeff[1][outputs][inputs][-1]
            	counter += 1
	# Constrain the controller
	# Interaction Index
	s = [[] for outputs in range(0,sys_size[1])] for inputs in range(0,sys_size[0])
	MS= s;
	for outputs in range(0,sys_size[1]):
		for inputs in range(0,sys_size[2]):
			s[outputs][inputs] = 0.1;
			MS[outputs][inputs] = np.sqrt(2);
	# We assume a First Order Model or a First Order Time Delay Model
	# Hence, we can calculate the coefficients of a PI by direct pole placement		
	# Assuming constant damping = 0.9 and constant omega0 = 0.5 (small influence of high frequency -> disturbance mostly high)
	damping = 0.9
	omega0 = 0.5
	P = [[] for outputs in range(0,sys_size[1])]

	# Wrong implementation! Calculate kI First and then calculate kP!!!
	for outputs in range(0,sys_size[1]):
		# Controller for the main loop
		inputs = outputs
		# Calculate a kP based on the 2nd Order Sensitivity Function
		# sys_coeff[1][inputs][outputs][1] is Ti and sys_coeff[0][inputs][outputs][-1] is k
		kP = np.abs((2*damping*omega0 - sys_coeff[1][inputs][outputs][0]) / sys_coeff[0][inputs][outputs][-1])
		#Calculate kI based on the non set point weighted interaction
		kI = np.abs(s[outputs][inputs])/np.abs(k[outputs][inputs]*MS[outputs][inputs]*MS[inputs][outputs]) - kP*omega0
		#kI = omega0**2 / sys_coeff[0][inputs][outputs][-1]
		P[outputs] = [kP, kI]
		


	if output is "C":
		return P, sys_pairing;
	else:
		# Make a Controller
		# Start by creating a zero array of same size as the system
		num = [[[0.0] for inputs in range(0,sys_size[2])] for outputs in range(0, sys_size[1])]
		den = [[[1.0] for inputs in range(0,sys_size[2])] for outputs in range(0, sys_size[1])]
		print(num, den)
		print(np.shape(num), np.shape(den))
		# Proportional Gain
		outputs = 0
		for inputs in sys_pairing:
				num[outputs][inputs] = [np.divide(P[outputs][0],P[outputs][1]),P[outputs][0]]
				outputs += 1
		# Integral Gain
		outputs = 0
		for inputs in sys_pairing:
			den[outputs][inputs] = [P[outputs][1], 0.]
			outputs += 1
		C = cn.tf(num,den)
		return C;
