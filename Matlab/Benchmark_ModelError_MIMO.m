%% Benchmarking used for Model Error

%% Hygiene
clear all
close all
clc

%% Parameter
Max_Order = 1;
Max_Sample = 1;
% Create Sum Time Constant
TSum = 10;
%% Benchmark

% Preallocate space

% Store the maximum Sensitivity
IAE = zeros(Max_Sample,2);
IAW = IAE;

% Outer Loop over the order of the System
for Systemorder = 1:Max_Order
   % Inner Loop over Samples
   for Sample = 1:Max_Sample
       %% Random FOTD Creation
       % Create gain of 1
       Num = 1;
       % Create Time Constant
       T = 50;
       % Create The Transfer Function
       G = tf('s');
       % Create the Model of the Plant
       
       for Inputs = 1:2
           L = (0.1+rand()*0.49)*T;
           for Outputs = 1:2
               TNow = T*(rand()*0.3+1);
               % Create Delay from 0.1*T <= L <= 0.6*T
               G(Inputs,Outputs) = tf(Num+rand(), [TNow,1],'OutputDelay',L);
            end
       end
       % Rosebroke Function
       %G = tf({1,2;1,1},{[1,1],[1,3];[1,1],[1,1]})

       %% PID Tuning Matlab
       Constrains = [0.1,0.1,sqrt(2),sqrt(2)];
       CFR = Decoupling_A(G,Constrains); % Returns a pid2 object
       C = tf(CFR);
       C1 = C(1);
       C2 = C(2);
       for Inputs = 1:2
           for Outputs = 1:2
               CF(Inputs,Outputs) = C1(:,:,Inputs,Outputs); % w -> u
               CR(Inputs,Outputs) = C2(:,:,Inputs,Outputs); % y -> u
           end
       end
       
       CL1 = CF*feedback(G,CR,+1);
       %% PID Tuning AMIGO
       CFR = Decoupling_A(G,Constrains,'AMIGO'); % Returns a pid2 object
       % Create the open loop
       % Get the feedback controller of C for the sensitivity
       C = tf(CFR);
       C1 = C(1);
       C2 = C(2);
       for Inputs = 1:2
           for Outputs = 1:2
               CF(Inputs,Outputs) = C1(:,:,Inputs,Outputs); % w -> u
               CR(Inputs,Outputs) = C2(:,:,Inputs,Outputs); % y -> u
           end
       end
       
       CL2 = CF*feedback(G,CR,+1);
       %% Simulate the Output
       % Ideal derivative
       D = tf([1,0],1);
       % Simulation Time
       dt = 0.1;
       time = 0:dt:1000;
       % Input with offset
       u_0 = ones(2,length(time));
       u_1 = zeros(size(u_0));
       u_2 = u_1;
       u_1(1,:) = ones(size(u_1(1,:)));
       u_2(2,:) = ones(size(u_2(2,:)));
       %u(1,:) = ones(1,length(u));       
       %u(2,400:end) = ones(1,length(u(:,400:end)));
       % Simulation of both inputs
       y1 = lsim(CL1,u_0,time);
       y2 = lsim(CL2,u_0,time);
       % Derive Absolut Integrated Error
       IAE(Sample,1) = dt*mean(sum(abs(u_0'-y1)))/time(end);
       IAE(Sample,2) = dt*mean(sum(abs(u_0'-y2)))/time(end);
       % Get the waviness
       y1 = lsim(D*CL1,u_0,time);
       y2 = lsim(D*CL2,u_0,time);
       % Integrated Waviness
       IAW(Sample,1) = dt*mean(sum(abs(y1)))/time(end);
       IAW(Sample,2) = dt*mean(sum(abs(y2)))/time(end);
       
       
   end       
   % Normalize the IAE to Maximum
   %IAE_max = max(max(IAE));
   %IAE = IAE./max(max(IAE));
   %IAW_max = max(max(IAW));
   %IAW = IAW./max(max(IAW));
   
end
