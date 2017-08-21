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
import Algorithms as alg


# Define an experiment
from sacred import Experiment
ex = Experiment()

@ex.config
def my_config():
	# Sample size per system order
	sample_size = 2
	# Maximum system order (between 1 and ...)
	system_order = 2
	# Gain Limit
	min_gain = 0.1
	max_gain = 10
	# Lag Limit 
	min_lag = 1
	max_lag = 10

@ex.automain 
def my_man(system_order,sample_size,min_gain,max_gain):
	# Runs randomsiso to evaluate
	from Randomsiso import RandomSys

	# Result Storage with 9 Kennzahlen
	R = []
	for sample in range(0,sample_size):
		for order in range(0,system_order):
			r = RandomSys.run(config_updates={'min_gain': min_gain, 'max_gain': max_gain})
			R.append(r.result)

	return(R)