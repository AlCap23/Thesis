# -*- coding: utf-8 -*-
"""
Created on Mon Sep 25 15:45:35 2017

@author: Michael
"""

from MoBASimulator import *
import pandas as pd
import csv
sim = Simulator()
sim.showLogWindow()
tambientstart=273;
tambientend=314;
tambientstep=5;

# Szenarien generieren
Szenarien=[]

for Qdot in [-50e3,-75e3,-90e3]:
    for inertiaFan in [10,20,30]:
        for inertiaValve in [10,20,30]:
            for Gascooler in [30,50,70]:
                for Zeta in [0.5,1.0,1.5]:
                    Szenarien.append({"Qdot":Qdot,"inertiaFan":inertiaFan,"inertiaValve":inertiaValve,"Gascooler":Gascooler,"Zeta":Zeta})


for i in range(0,60):
    K_Fan_Temperature=[]
    T_Fan_Temperature=[]
    L_Fan_Temperature=[]
    
    K_Fan_Pressure=[]
    T_Fan_Pressure=[]
    L_Fan_Pressure=[]
    
    K_Valve_Temperature=[]
    T_Valve_Temperature=[]
    L_Valve_Temperature=[]
    
    K_Valve_Pressure=[]
    T_Valve_Pressure=[]
    L_Valve_Pressure=[]
        
    for tambient in range(tambientstart,tambientend,tambientstep):
       
        sim.loadModel('Identification_Fan_Temperature.fmu')
        sim.set({'fmu.TUmgebung':tambient})
        sim.set({'fmu.Qdotkaelte':Szenarien[i]["Qdot"]})
        sim.set({'fmu.System.firstOrderFan.T':Szenarien[i]["inertiaFan"]})
        sim.set({'fmu.System.firstOrderValve.T':Szenarien[i]["inertiaValve"]})
        sim.set({'fmu.System.Gascooler.hxGeometry.nParallelTubes':Szenarien[i]["Gascooler"]})
        sim.set({'fmu.System.zeta':Szenarien[i]["Zeta"]})
               
        res=sim.simulate(10,350000);
        K=res['fmu.K_Fan_Temperature'][-1]
        K_Fan_Temperature.append(K)
        T=res['fmu.T_Fan_Temperature'][-1]
        T_Fan_Temperature.append(T)
        L=res['fmu.L_Fan_Temperature'][-1]
        L_Fan_Temperature.append(L)
        sim.resetModelState()
        sim.reloadModel();
        
        sim.loadModel('Identification_Fan_Pressure.fmu')
        sim.set({'fmu.TUmgebung':tambient})
        sim.set({'fmu.Qdotkaelte':Szenarien[i]["Qdot"]})
        sim.set({'fmu.System.firstOrderFan.T':Szenarien[i]["inertiaFan"]})
        sim.set({'fmu.System.firstOrderValve.T':Szenarien[i]["inertiaValve"]})
        sim.set({'fmu.System.Gascooler.hxGeometry.nParallelTubes':Szenarien[i]["Gascooler"]})
        sim.set({'fmu.System.zeta':Szenarien[i]["Zeta"]})
        
        res=sim.simulate(10,350000);
        K=res['fmu.K_Fan_Pressure'][-1]
        K_Fan_Pressure.append(K)
        T=res['fmu.T_Fan_Pressure'][-1]
        T_Fan_Pressure.append(T)
        L=res['fmu.L_Fan_Pressure'][-1]
        L_Fan_Pressure.append(L)
        sim.resetModelState()
        sim.reloadModel();
        
        sim.loadModel('Identification_Valve_Temperature.fmu')
        sim.set({'fmu.TUmgebung':tambient})
        sim.set({'fmu.Qdotkaelte':Szenarien[i]["Qdot"]})
        sim.set({'fmu.System.firstOrderFan.T':Szenarien[i]["inertiaFan"]})
        sim.set({'fmu.System.firstOrderValve.T':Szenarien[i]["inertiaValve"]})
        sim.set({'fmu.System.Gascooler.hxGeometry.nParallelTubes':Szenarien[i]["Gascooler"]})
        sim.set({'fmu.System.zeta':Szenarien[i]["Zeta"]})
        
        res=sim.simulate(10,350000);
        K=res['fmu.K_Valve_Temperature'][-1]
        K_Valve_Temperature.append(K)
        T_=res['fmu.T_Valve_Temperature'][-1]
        T_Valve_Temperature.append(T)
        L=res['fmu.L_Valve_Temperature'][-1]
        L_Valve_Temperature.append(L)
        sim.resetModelState()
        sim.reloadModel();
        
        sim.loadModel('Identification_Valve_Pressure.fmu')
        sim.set({'fmu.TUmgebung':tambient})
        sim.set({'fmu.Qdotkaelte':Szenarien[i]["Qdot"]})
        sim.set({'fmu.System.firstOrderFan.T':Szenarien[i]["inertiaFan"]})
        sim.set({'fmu.System.firstOrderValve.T':Szenarien[i]["inertiaValve"]})
        sim.set({'fmu.System.Gascooler.hxGeometry.nParallelTubes':Szenarien[i]["Gascooler"]})
        sim.set({'fmu.System.zeta':Szenarien[i]["Zeta"]})
        
        res=sim.simulate(10,350000);
        K=res['fmu.K_Valve_Pressure'][-1]
        K_Valve_Pressure.append(K)
        T=res['fmu.T_Valve_Pressure'][-1]
        T_Valve_Pressure.append(T)
        L=res['fmu.L_Valve_Pressure'][-1]
        L_Valve_Pressure.append(L)
        sim.resetModelState()
        sim.reloadModel();
        
 
    K_Fan_Temperature_Text="";
    T_Fan_Temperature_Text="";
    L_Fan_Temperature_Text="";
    
    K_Fan_Pressure_Text="";
    T_Fan_Pressure_Text="";
    L_Fan_Pressure_Text="";
   
    K_Valve_Temperature_Text="";
    T_Valve_Temperature_Text="";
    L_Valve_Temperature_Text="";
    
    K_Valve_Pressure_Text="";
    T_Valve_Pressure_Text="";
    L_Valve_Pressure_Text="";
   
    for j in range(0,len(K_Fan_Temperature)):
        K_Fan_Temperature_Text=str(K_Fan_Temperature_Text)+"  "+str(K_Fan_Temperature[j])
        T_Fan_Temperature_Text=str(T_Fan_Temperature_Text)+"  "+str(T_Fan_Temperature[j])
        L_Fan_Temperature_Text=str(L_Fan_Temperature_Text)+"  "+str(L_Fan_Temperature[j])
        
    K_Fan_Temperature_Text=str(K_Fan_Temperature_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    T_Fan_Temperature_Text=str(T_Fan_Temperature_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    L_Fan_Temperature_Text=str(L_Fan_Temperature_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    
    filename="Fan_Temperature_K.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([K_Fan_Temperature_Text])

    filename="Fan_Temperature_T.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([T_Fan_Temperature_Text])

    filename="Fan_Temperature_L.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([L_Fan_Temperature_Text])
  
   
    for j in range(0,len(K_Fan_Pressure)):
        K_Fan_Pressure_Text=str(K_Fan_Pressure_Text)+"  "+str(K_Fan_Pressure[j])
        T_Fan_Pressure_Text=str(T_Fan_Pressure_Text)+"  "+str(T_Fan_Pressure[j])
        L_Fan_Pressure_Text=str(L_Fan_Pressure_Text)+"  "+str(L_Fan_Pressure[j])
        
    K_Fan_Pressure_Text=str(K_Fan_Pressure_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    T_Fan_Pressure_Text=str(T_Fan_Pressure_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    L_Fan_Pressure_Text=str(L_Fan_Pressure_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    
    filename="Fan_Pressure_K.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([K_Fan_Pressure_Text])
    filename="Fan_Pressure_T.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([T_Fan_Pressure_Text])
    filename="Fan_Pressure_L.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([L_Fan_Pressure_Text])
        
        
    for j in range(0,len(K_Valve_Temperature)):
        K_Valve_Temperature_Text=str(K_Valve_Temperature_Text)+"  "+str(K_Valve_Temperature[j])
        T_Valve_Temperature_Text=str(T_Valve_Temperature_Text)+"  "+str(T_Valve_Temperature[j])
        L_Valve_Temperature_Text=str(L_Valve_Temperature_Text)+"  "+str(L_Valve_Temperature[j])
        
    K_Valve_Temperature_Text=str(K_Valve_Temperature_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    T_Valve_Temperature_Text=str(T_Valve_Temperature_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    L_Valve_Temperature_Text=str(L_Valve_Temperature_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    
    filename="Valve_Temperature_K.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([K_Valve_Temperature_Text])
    filename="Valve_Temperature_T.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([T_Valve_Temperature_Text])
    filename="Valve_Temperature_L.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([L_Valve_Temperature_Text])
               
        
    for j in range(0,len(K_Valve_Pressure)):
        K_Valve_Pressure_Text=str(K_Valve_Pressure_Text)+"  "+str(K_Valve_Pressure[j])
        T_Valve_Pressure_Text=str(T_Valve_Pressure_Text)+"  "+str(T_Valve_Pressure[j])
        L_Valve_Pressure_Text=str(L_Valve_Pressure_Text)+"  "+str(L_Valve_Pressure[j])
        
    K_Valve_Pressure_Text=str(K_Valve_Pressure_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    T_Valve_Pressure_Text=str(T_Valve_Pressure_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    L_Valve_Pressure_Text=str(L_Valve_Pressure_Text)+"  "+str(Szenarien[i]["Qdot"])+"  "+str(Szenarien[i]["inertiaFan"])+"  "+str(Szenarien[i]["inertiaValve"])+"  "+str(Szenarien[i]["Gascooler"])+"  "+str(Szenarien[i]["Zeta"])
    
    filename="Valve_Pressure_K.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([K_Valve_Pressure_Text])
    filename="Valve_Pressure_T.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([T_Valve_Pressure_Text])
    filename="Valve_Pressure_L.csv"
    with open(filename,'ab') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow([L_Valve_Pressure_Text])
        
        
    
        
    
    