%%

%% Preprocessing

clear all
close all
clc

% Add path for functions
addpath('C:\Users\juliu\Documents\GIT\New folder\Matlab')


%% Define TF
%  Rosenbrook Function as an example for a MIMO TF without Delay
G = tf({12.8,-18.9;6.6,-19.4},{[16.7,1],[21,1];[10.9,1],[14.4,1]},'IODelay',[1,3;7,3]);
%% Decouple via RGA
C1 = Decoupling_RGA(G);
% Closed Loop
CL1 = feedback(G*C1,eye(2));

%% Decouple via Aström
C2 = Decoupling_A(G,[0.2, 0.2, sqrt(2), sqrt(2)],'AMIGO');
% Preprocess PID2 Object -> Set Point Weight
C = tf(C2); % Convert to TF
CA = C(1); % Set Point Controller
CB = C(2); % Feedback Controller
for Inputs = 1:2
    for Outputs = 1:2
        CR(Inputs,Outputs) = CA(:,:,Inputs,Outputs); % w -> u
        CY(Inputs,Outputs) = CB(:,:,Inputs,Outputs); % y -> u
    end
end
% Closed Loop 
CL2 = CR*feedback(G,CY,+1);
%% Decouple via Modified Aström
C3 = Decoupling_FOTD(G,[0.2, 0.2, sqrt(2), sqrt(2)]);
% Preprocess PID2 Object -> Set Point Weight
C = tf(C2); % Convert to TF
CA = C(1); % Set Point Controller
CB = C(2); % Feedback Controller
for Inputs = 1:2
    for Outputs = 1:2
        CR(Inputs,Outputs) = CA(:,:,Inputs,Outputs); % w -> u
        CY(Inputs,Outputs) = CB(:,:,Inputs,Outputs); % y -> u
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