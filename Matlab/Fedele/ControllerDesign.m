%%

%% Preprocessing

clear all
close all
clc
%% Preparation

% Input
% Filename for saving
%filename = 'Data_80kW_T273.mat';
% Operating point
Current = 1;
% Constrains
Constrains = [0.1,0.5,sqrt(2),sqrt(2)];

% Add path for functions -> Windows only
%addpath('C:\Users\juliu\Documents\GIT\New folder\Matlab');
%addpath('C:\Users\juliu\Documents\GIT\New folder\Matlab\KTL_5000');
% Get the information from Simulation study
load('Fedele_Minimal.mat');

% Get the TF Data
OP = TFData.OperatingPoint(Current);
KV = [TFData.Fan_Temperature(1,Current),TFData.Valve_Temperature(1,Current);...
      TFData.Fan_Pressure(1,Current),TFData.Valve_Pressure(1,Current)];
TV = [TFData.Fan_Temperature(2,Current),TFData.Valve_Temperature(2,Current);...
      TFData.Fan_Pressure(2,Current),TFData.Valve_Pressure(2,Current)];
LV = [TFData.Fan_Temperature(3,Current),TFData.Valve_Temperature(3,Current);...
      TFData.Fan_Pressure(3,Current),TFData.Valve_Pressure(3,Current)];

 %% Make a transfer Function
G = tf('s');
for Inputs = 1:2
    for Outputs = 1:2
        G(Outputs,Inputs) = tf(KV(Outputs,Inputs),[TV(Outputs,Inputs),1],'IODelay',LV(Outputs,Inputs));
    end
end

G.InputName = {'Fan';'Valve'};
G.OutputName = {'Temperature','Pressure'};

%% Make a Model for Smith Predictor
% Delay Free Model
G_M = G;
G_M.IODelay = zeros(size(G));
% Delay Model 
G_D = tf([1,1;1,1]);
G_D.IODelay = G.IODelay;
%% Decouple via RGA
C1 = Decoupling_A(G,Constrains,'AMIGO',0);
%C1 = Decoupling_RGA(G,Constrains,'DC',0);
%C1 = Decoupling_FOTD(G,Constrains,'AMIGO',0);
% Preprocess PID2 Object -> Set Point Weight
C = tf(C1); % Convert to TF
CA = C(1); % Set Point Controller
CB = C(2); % Feedback Controller
for Inputs = 1:2
    for Outputs = 1:2
        CR(Outputs,Inputs) = CA(:,:,Outputs,Inputs); % w -> u
        CY(Outputs,Inputs) = CB(:,:,Outputs,Inputs); % y -> u
    end
end
% Store the Controller
%CR1 = CR
%CY1 = CY
% Closed Loop 
CL1 = feedback(G,CY,+1)*CR;
CL1.InputName = {'Fan';'Valve'};
CL1.OutputName = {'Temperature';'Pressure'};
% Sensitivity
S1 = inv(eye(2)-G*CY);
OL1 = -G*CY;
%% Decouple via Astr�m
C2 = Decoupling_A2(G,Constrains,'AMIGO',0);
% Preprocess PID2 Object -> Set Point Weight
C = tf(C2); % Convert to TF
CA = C(1); % Set Point Controller
CB = C(2); % Feedback Controller
for Inputs = 1:2
    for Outputs = 1:2
        CR(Outputs,Inputs) = CA(:,:,Outputs,Inputs); % w -> u
        CY(Outputs,Inputs) = CB(:,:,Outputs,Inputs); % y -> u
    end
end
% Closed Loop 
CL2 = feedback(G,CY,+1)*CR;
CL2.InputName = {'Fan';'Valve'};
CL2.OutputName = {'Temperature';'Pressure'};
%Sensitivity
S2 = inv(eye(2)-G*CY);
OL2 = -G*CY;
%% Decouple via Modified Astr�m
C3 = Decoupling_F(G,Constrains,'AMIGO',0);
% Preprocess PID2 Object -> Set Point Weight
C = tf(C3); % Convert to TF
CA = C(1); % Set Point Controller
CB = C(2); % Feedback Controller
for Inputs = 1:2
    for Outputs = 1:2
        CR(Outputs,Inputs) = CA(:,:,Outputs,Inputs); % w -> u
        CY(Outputs,Inputs) = CB(:,:,Outputs,Inputs); % y -> u
    end
end

% Closed Loop
CL3 = feedback(G,CY,+1)*CR;
CL3.InputName = {'Fan';'Valve'};
CL3.OutputName = {'Temperature';'Pressure'};
%Sensitivity
S3 = inv(eye(2)-G*CY);
OL3 = -G*CY;
%% Mixing
% % Since the temperature control is faster from G -> Use G
% CR(:,1) = CR3(:,1);
% CY(:,1) = CY3(:,1);
% % Use RGA for Pressure control
% CR(:,2) = CR1(:,2);
% CY(:,2) = CY1(:,2);
% % Close the loop
% CL4 = CR*feedback(G,CY,+1);
% CL4.InputName = {'Fan';'Valve'};
% CL4.OutputName = {'Temperature';'Pressure'};
%% Get Results for Step Response
figure(1)
step(CL1)
hold on
grid on
step(CL2)
step(CL3)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')

% Simulate
time = 0:1:6000 ; % Define a time array
u = zeros(2,length(time)); % Define input array
u(1,:) = 1; % temperature step at t= 0
u(2,3001:end) = -1; % pressure step at t = 3001

y1 = lsim(CL1,u,time); % RGA System
y2 = lsim(CL2,u,time); % Q Decoupling
y3 = lsim(CL3,u,time); % G Decoupling

%% Export
%save(filename,'y1','y2','y3','u','time');
%% Robustness Analysis
figure(2)
title('Singular Values of the Sensitivity Functions')
sigma(S1)
hold on
grid on
sigma(S2)
sigma(S3)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')

% Condition Number
[sv1,f1] = sigma(CL1);
[sv2,f2] = sigma(CL2);
[sv3,f3] = sigma(CL3);

% Loop over frequency
for currentf = 1:length(sv1)
   cn1(currentf) = max(sv1(:,currentf))/min(sv1(:,currentf));
end

for currentf = 1:length(sv2)
   cn2(currentf) = max(sv2(:,currentf))/min(sv2(:,currentf));
end

for currentf = 1:length(sv3)
   cn3(currentf) = max(sv3(:,currentf))/min(sv3(:,currentf));
end

figure(3)
title('Condition Number of the Systems')
loglog(f1,cn1,'o-')
hold on
loglog(f2,cn2,'o-')
loglog(f3,cn3,'o-')
grid on
legend('RGA','Decoupling with Q Design','Decoupling with G Design')
%% Define a Test Simulation - Constant Temperature
time = 0:1:10000; % Time
u = zeros(2,length(time)); % Set Point
u(1,:) = 0; % Constant Temperature
u(2,1:200) = 0; % Pressure
u(2,200:4999) = 1;
u(2,5000:end) = -1;

figure(4)
lsim(CL1,u,time)
hold on
grid on
lsim(CL2,u,time)
lsim(CL3,u,time)
%lsim(CL4,u,time)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')

%% Define a Test Simulation - Constant Pressure
u = zeros(2,length(time)); % Set Point
u(1,1:200) = 0; % Vary Temperature
u(1,200:4999) = 1; 
u(1,5000:end) = -1;
u(2,:) = 0; % Constant Pressure

figure(5)
lsim(CL1,u,time)
hold on
grid on
lsim(CL2,u,time)
lsim(CL3,u,time)
%lsim(CL4,u,time)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')
