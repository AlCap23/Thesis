%% Benchmarking used for Benchmarking Aström Decoupling

%% Hygiene
clear all
close all
clc

%% Parameter
Max_Order = 1;
Max_Sample = 10;
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
       for Inputs = 1:2
           for Outputs = 1:2
               TNow = T*(rand()*0.5+1);
               % Create Delay from 0.1*T <= L <= 0.6*T
               L = 0.6*TNow;
               G(Inputs,Outputs) = tf(Num+rand(), [TNow,1],'IODelay',L);
               G(Inputs,Outputs) = G(Inputs,Outputs)^Systemorder;
           end
       end
       % Create  Controller
       C = tf('s');
       Constrains = [0.1,0.1,sqrt(2),sqrt(2)];
       C = Test(G,Constrains,'AMIGO'); % Returns a pid2 object
       % Create the open loop
       % Get the feedback controller of C for the sensitivity
       C = tf(C(:,:));
       C = C(2);
       for Inputs = 1:2
           for Outputs = 1:2
               CF(Inputs,Outputs) = C(:,:,Inputs,Outputs);
           end
       end
       OL = series(pade(G,8),CF);
       % Get the Stability Information in form of the margins
       % Store the data
       % Maximum Sensitivity
       S = inv(eye(2)+OL); % Sensitivity for real system
       % Get the maximum Gain
       fband = [0,200];
       MS(Systemorder,Sample,:) = [getPeakGain(S,0.01,fband), 0];
       
       
       end
end
