# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
# Import the needed Packages
# FMU Simulation
import MoBASimulator as mb
# Numpy
import numpy as np
# Bokeh for Plotting
import bokeh.plotting as bk
import bokeh.io as bi
#from bokeh.io import export_svgs
#bi.output_notebook()
# Algorithms
import Algorithms as alg

## Model Parameter
#k = np.random.uniform(-10,10,(2,2))
#k[k==0] = 1e-10
#t = np.random.uniform(1,5,(2,2))
#l = np.random.uniform(1,5,(2,2))


# Rosenbrock
k = [[1,2./3.],[1,1]]
t = [[1,1./3.],[1,1]]
l = [[1e-10,1e-10],[1e-10,1e-10]]

# Woodberry
#k = [[12.8,-18.9],[6.6,-19.4]]
#t = [[16.7,21.0],[10.9,14.4]]
#l = [[1,3],[7,3]]

# Woodberry
#k = [[12.8,2],[5,-19.4]]
#t = [[16.7,21.0],[10.9,14.4]]
#l = [[1e-10,5],[1e-10,1e-10]]


# The needed Parameter
K = np.zeros((2,2))
T = np.zeros((2,2))
L = np.zeros((2,2))

# Load a Model
sim = mb.Simulator()
sim.clear()
sim.loadModel("C:/Users/juliu/Documents/Thesis/Modelica/FMU/2_2/Masterthesis_Models_mimo_0processmodel.fmu")
#sim.showParameterDialog()
# Show log window
sim.showLogWindow()

# Parameter Values
params = {}
# Loop over system size
for outputs in range(1,3):
    for inputs in range(1,3):
        # Process Parameter 
        # System Gain
        params.update({"fmu.num["+str(outputs)+","+str(inputs)+",1]": k[outputs-1][inputs-1]})
        # System Lag
        params.update({"fmu.den["+str(outputs)+","+str(inputs)+",1]": t[outputs-1][inputs-1]})
        params.update({"fmu.den["+str(outputs)+","+str(inputs)+",2]": 1})
        # System Delay
        params.update({"fmu.delay["+str(outputs)+","+str(inputs)+"]": l[outputs-1][inputs-1]})

# Set Parameter and show for checking
sim.set(params)



# First run, Input 1 -> Output 1 & 2
sim.set({"fmu.u[1]": 1,"fmu.u[2]": 0})
# Set timestep = 1e-2, endtime = 100
sim.resetModelState()
res=sim.simulate(0.1, 1000)

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
res=sim.simulate(0.1, 20000)

# Get the signals
y = res["fmu.y[1]"]
y2 = res["fmu.y[2]"]
u = res["fmu.u[2]"]
time = res["time"]

# Get TF from Input 2 to Output 1
K[0][1],T[0][1],L[0][1] = alg.Integral_Identification(y,u,time)
# Get TF from Input 2 to Output 2
K[1][1],T[1][1],L[1][1] = alg.Integral_Identification(y2,u,time)


for methods in ["RGA","Aström","Decoupling"]:
    # Make a controller
    if methods == "RGA":
        KY,B,D = alg.Control_Decentral(K,T,L)
    elif methods == "Aström":
        KY,B,D = alg.Control_Astrom(K,T,L,1e5*np.eye(2,2))
    elif methods == "Decoupling":
        KY,B,D = alg.Control_Decoupled(K,T,L,1e5*np.eye(2,2))

    # Make zeros to 1e-10 for numerical stable process
    KY[KY==0] = 1e-10
    B[B==0] = 1e-10
    D[D==0] = 1e-10


    # Load the closed loop model
    sim.clear()
    sim.loadModel("C:/Users/juliu/Documents/Thesis/Modelica/FMU/2_2/Masterthesis_Models_mimo_0closedloop.fmu")
    #sim.showParameterDialog()
    # Parameter Values
    params = {}
    # Loop over system size
    for outputs in range(1,3):
        for inputs in range(1,3):
            # Controller Parameter 
            # Proportional Gain
            params.update({"fmu.kp["+str(outputs)+","+str(inputs)+"]": KY[outputs-1][inputs-1][0]})
             # Integral Gain
            params.update({"fmu.ki["+str(outputs)+","+str(inputs)+"]": KY[outputs-1][inputs-1][1]})
            # Decoupler
            params.update({"fmu.d["+str(outputs)+","+str(inputs)+"]": D[outputs-1][inputs-1]})
            # Set Point Weight
            params.update({"fmu.b["+str(outputs)+","+str(inputs)+"]": B[outputs-1][inputs-1]})
            # Process Parameter 
            # System Gain
            params.update({"fmu.num["+str(outputs)+","+str(inputs)+",1]": k[outputs-1][inputs-1]})
            # System Lag
            params.update({"fmu.den["+str(outputs)+","+str(inputs)+",1]": t[outputs-1][inputs-1]})
            params.update({"fmu.den["+str(outputs)+","+str(inputs)+",2]": 1})
            # System Delay
            params.update({"fmu.delay["+str(outputs)+","+str(inputs)+"]": l[outputs-1][inputs-1]})
    # Set Parameter Values
    sim.set(params)
    #sim.showParameterDialog()
    
    # First run, Input 1 -> Output 1 & 2
    sim.resetModelState()
    sim.set({"fmu.u[1]": 1,"fmu.u[2]": 0})
    # Set timestep = 1e-2, endtime = 100
    res=sim.simulate(0.05, 500)
    sim.set({"fmu.u[1]": 1,"fmu.u[2]": 1})
    # Set timestep = 1e-2, endtime = 100
    res=sim.simulate(0.05, 1000)
    
    
    # Make a plot
    from bokeh.models import LinearAxis, Range1d
    from bokeh.layouts import gridplot
    
    p1 = bk.figure(title = "Rosenbrocks System - "+methods)
    p1.line(res["time"],res["fmu.y[1]"])
    
    p1.extra_y_ranges = {"2": Range1d(start=-.1, end=1.1)}
    p1.add_layout(LinearAxis(y_range_name="2"), 'right')
    p1.line(res["time"],res["fmu.u[1]"], color="black", y_range_name="2")
    
    
    
    p2 = bk.figure(title = "Rosenbrocks System - "+methods)
    p2.line(res["time"],res["fmu.y[2]"])
    
    p2.extra_y_ranges = {"2": Range1d(start=-.1, end=1.1)}
    p2.add_layout(LinearAxis(y_range_name="2"), 'right')
    p2.line(res["time"],res["fmu.u[2]"], color="black", y_range_name="2")
    
    p = gridplot([[p1],[p2]])
    bk.show(p)