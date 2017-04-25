%% Benchmarking used for Model Error

%% Hygiene
clear all
close all
clc

%% Parameter
Max_Order = 9;
Max_Sample = 100;
% Create Sum Time Constant
TSum = 250;
%% Benchmark

% Preallocate space

% Store the maximum Sensitivity
MS = zeros(Max_Order,Max_Sample,2);
ACC = zeros(Max_Order,Max_Sample);


% Outer Loop over the order of the System
for Systemorder = 1:Max_Order
   % Inner Loop over Samples
   for Sample = 1:Max_Sample
       % Create gain of 1
       Num = 1;
       % Create Time Constant
       T = TSum/Systemorder;
       % Create The Transfer Function
       G = tf('s');
       % Create the Model of the Plant
       G_M = tf('s');
       for Inputs = 1:2
           for Outputs = 1:2
               TNow = T*(rand()*0.5+1);
               % Create Delay from 0.1*T <= L <= 0.6*T
               L = 0.6*TNow;
               G(Inputs,Outputs) = tf(Num+rand(), [TNow,1],'InputDelay',L);
               G(Inputs,Outputs) = G(Inputs,Outputs)^Systemorder;
           end
       end
             
       for Inputs = 1:2
           for Outputs = 1:2
               G_M(Inputs,Outputs) = Relay_Identification(G(Inputs,Outputs));                
           end
       end
       
       % Create the FOTD
       C = tf('s');
       % Create  Controller
       Constrains = [0.1,0.1,sqrt(2),sqrt(2)]
       CL = Decoupling_A(G_M,Constrains)
       % Create the open loop
       OL = pade(G,10)*C;
       OLM = pade(G_M,10)*C;
       % Get the Stability Information in form of the margins
       % Store the data
       % Maximum Sensitivity
       S = inv(eye(2)+OL); % Sensitivity for real system
       S_M = inv(eye(2)+OLM); % Sensitivity for ideal system
       % Get the maximum Gain
       MS(Systemorder,Sample,:) = [getPeakGain(S), getPeakGain(S_M)];
       
       
       end
end
