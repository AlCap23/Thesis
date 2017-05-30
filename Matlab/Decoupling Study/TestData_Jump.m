%% TestData_Jump
%  Based on the Test Data from Michael Nödings visit on 2017/05/16
%  
%% Preprocessing
clear all
close all
clc

% Filename for saving
filename = 'Data_Jump_20170516.mat';
% Store the Coupling Factor for static system -> See Lunze II, p.194, Eq. 4.82
% Store the step Information
Info = struct('RGA',1,'Q',1,'G',1,'U',1,'Y1',1,'Y2',1,'Y3',1,'TIME',1);



%% Initialize the TF
% Implement the Data
G = tf({-1,0.2;0.0725,-0.04},...
    {[45,1],[45,1];[35,1],[27,1]},...
    'IODelay',[13,45;45,10],...
    'InputName',{'Fan';'Valve'},...
    'OutputName',{'Temperature','Pressure'})

%% Decouple via RGA
C1 = Decoupling_RGA(G);
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
CL1 = feedback(G,CY,+1)*CR;
%% Decouple via Astr�m
C2 = Decoupling_A(G,[0.1, 0.5, sqrt(2), sqrt(2)],'AMIGO',0);
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
CL2 = feedback(G,CY,+1)*CR;
%% Decouple via Modified Astr�m
C3 = Decoupling_FOTD(G,[0.1, 0.5, sqrt(2), sqrt(2)],'AMIGO',0);
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
CL3 = feedback(G,CY,+1)*CR;
%% Get the information
Info.RGA = stepinfo(CL1);
Info.Q = stepinfo(CL2);
Info.G = stepinfo(CL3);

save(filename,'Info')

%% Plot the Data
step(CL1)
grid on
hold on
step(CL2)
step(CL3)


% Simulate
time = 0:1:2000 ; % Define a time array
u = zeros(2,length(time)); % Define input array
u(1,:) = 1; % temperature step at t= 0
u(2,1001:end) = -1; % pressure step at t = 3001

y1 = lsim(CL1,u,time); % RGA System
y2 = lsim(CL2,u,time); % Q Decoupling
y3 = lsim(CL3,u,time); % G Decoupling

Info.U = u;
Info.Y1 = y1;
Info.Y2 = y2;
Info.Y3 = y3;
Info.TIME = time;

save(filename,'Info','y1','y2','y3','u','time')
