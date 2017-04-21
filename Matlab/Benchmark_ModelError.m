%% Benchmarking used for Model Error

%% Hygiene
clear all
close all
clc

%% Parameter
Max_Order = 9;
Max_Sample = 200;
% Create Sum Time Constant
TSum = 200+randi([-50,50]);
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
       % Create Delay from 0.1*T <= L <= 0.6*T
       L = (rand*0.5+0.1)*T;
       % Create The Transfer Function
       G = tf(Num, [T,1],'OutputDelay',L);
       % Create High Systemorder
       G = G^Systemorder;
       % Create the FOTD
       G_M = Relay_Identification(G);
       % Get the Accuracy of the Model
       ss_acc = dcgain(G_M)/dcgain(G); % Steady State accuracy
       l_acc = G_M.OutputDelay / G.OutputDelay; % Time Delay accuracy
       t_acc = G_M.Denominator{:,:}(end-1) / G.Denominator{:,:}(end-1); % Time Constant accuracy
       w = [1,1,1]; % weights
       ACC(Systemorder,Sample) = mean(w.*[ss_acc,t_acc,l_acc]);       
       % Create a controller based on the model
       C = AMIGO_Tune(G_M,'PI');
       % Create the open loop
       OL = G*C;
       OLM = G_M*C;
       % Get the Stability Information in form of the margins
       % Store the data
       % Maximum Sensitivity
       S = pade(1/(1+OL),10); % Sensitivity for real system
       S_M = pade(1/(1+OLM),10); % Sensitivity for ideal system
       % Get the maximum Gain
       MS(Systemorder,Sample,:) = [getPeakGain(S), getPeakGain(S_M)];
       
       
       end
end
