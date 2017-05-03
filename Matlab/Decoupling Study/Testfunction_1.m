%%

%% Preprocessing

clear all
close all
clc

% Add path for functions
addpath('C:\Users\juliu\Documents\GIT\New folder\Matlab')


%% Define TF
%  Wood-Berry's Binary Distillation Column with just output delay as an example for a MIMO TF without Delay
G = tf({2,-1;1,3},{[5,1],[7,1];[10,1],[8,1]},'IODelay',[0,0;0,0]);
%% Decouple via RGA
C1 = Decoupling_RGA(G);
% Closed Loop
CL1 = feedback(G*C1,eye(2));

%% Decouple via Astr�m
C2 = Decoupling_A(G,[0.2, 0.2, sqrt(2), sqrt(2)],'AMIGO',0);
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
CL2 = CR*feedback(G,CY,+1);
%% Decouple via Modified Astr�m
C3 = Decoupling_FOTD(G,[0.2, 0.2, sqrt(2), sqrt(2)],'AMIGO',0);
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
CL3 = CR*feedback(G,CY,+1);
%% Get Results
figure()
step(CL1)
hold on
grid on
step(CL2)
step(CL3)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')