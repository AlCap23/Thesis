%%

%% Preprocessing

clear all
close all
clc

% Add path for functions
addpath('C:\Users\juliu\Documents\GIT\New folder\Matlab')


%% Define TF
%  Wood-Berry's Binary Distillation Column with just output delay as an example for a MIMO TF without Delay
G = tf({-1.02,22;-2.78,-8.4},...
    {[138,1],[92,1];[298,1],[120,1]},...
    'IODelay',[97,88;36,29],...
    'InputName',{'Fan';'Valve'},...
    'OutputName',{'Temperature','Pressure'});
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
% Closed Loop 
CL1 = CR*feedback(G,CY,+1);

%% Decouple via Astr�m
C2 = Decoupling_A(G,[0.1, 0.1, sqrt(2), sqrt(2)],'AMIGO',0);
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
C3 = Decoupling_FOTD(G,[0.1, 0.1, sqrt(2), sqrt(2)],'AMIGO',0);
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
%% Decouple via Modified Astr�m
C4 = Decoupling_D(G,[0.1, 0.1, sqrt(2), sqrt(2)],'AMIGO',0);
% Preprocess PID2 Object -> Set Point Weight
C = tf(C4); % Convert to TF
CA = C(1); % Set Point Controller
CB = C(2); % Feedback Controller
for Inputs = 1:2
    for Outputs = 1:2
        CR(Outputs,Inputs) = CA(:,:,Outputs,Inputs); % w -> u
        CY(Outputs,Inputs) = CB(:,:,Outputs,Inputs); % y -> u
    end
end
GD = G;
GD.IODelay = zeros(2,2);
% Get the gamma Matrix
gamma = [1, -GD(1,2)/GD(1,1);-GD(2,1)/GD(2,2),1]
% Closed Loop
CL4 = (gamma*CR)*feedback(G,gamma*CY,+1);
%% Get Results
figure()
step(CL1)
hold on
grid on
step(CL2)
step(CL3)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')