%%

%% Preprocessing

clear all
close all
clc

% Add path for functions -> Windows only
%addpath('C:\Users\juliu\Documents\GIT\New folder\Matlab')


%% Define TF
%  MIMO TF without coupling and no time delay -> RGA == ASTRÖM == FOTD
G = tf({-7.4,0;0,-14},...
    {[212,1],[1];[1],[209,1]},...
    'IODelay',[0,0;0,0],...
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
S1 = inv(eye(2)+G*CY);
T1 = inv(eye(2)+G*CY)*G*CR;
CL1 = feedback(G,CY,+1)*CR;
%% Decouple via Astr�m
C2 = Decoupling_A(G,[0.1, 0.1, 2, 2],'AMIGO',0);
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
S2 = inv(eye(2)+G*CY);
T2 = inv(eye(2)+G*CY)*G*CR;
CL2 = feedback(G,CY,+1)*CR;
%% Decouple via Modified Astr�m
C3 = Decoupling_FOTD(G,[0.1, 0.1, 2, 2],'AMIGO',0);
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
S3 = inv(eye(2)+G*CY);
T3 = inv(eye(2)+G*CY)*G*CR;
CL3 = feedback(G,CY,+1)*CR;

%% Get Results
figure(1)
step(CL1)
hold on
grid on
step(CL2)
step(CL3)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')
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