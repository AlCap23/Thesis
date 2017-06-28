%%

%% Preprocessing

clear all
close all
clc

% Add path for functions Windows
%addpath('C:\Users\juliu\Documents\GIT\New folder\Matlab')


%% Define TF
%  Rosenbrook Function as an example for a MIMO TF without Delay
% [1,3;7,3]
G = tf({12.8,-18.9;6.6,-19.4},{[16.7,1],[21,1];[10.9,1],[14.4,1]},'IODelay',[1,3;7,3]);
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
CL1 = feedback(G,CY,+1)*CR;

%% Decouple via Automatic Aström
C2 = Decoupling_A(G,[0.05, 0.3, sqrt(2), sqrt(2)],'AMIGO');
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
CL2 = feedback(G,CY,+1)*CR;
%% Decouple via Modified Aström
C3 = Decoupling_F(G,[0.05, 0.3, sqrt(2), sqrt(2)],'AMIGO');
% Preprocess PID2 Object -> Set Point Weight
C = tf(C3); % Convert to TF
CA = C(1); % Set Point Controller
CB = C(2); % Feedback Controller
for Inputs = 1:2
    for Outputs = 1:2
        CR(Inputs,Outputs) = CA(:,:,Inputs,Outputs); % w -> u
        CY(Inputs,Outputs) = CB(:,:,Inputs,Outputs); % y -> u
    end
end
% Closed Loop 
CL3 = feedback(G,CY,+1)*CR;

% %% Decouple via Aström Paper
% % For Comparision
% 
% % Get D
% D = inv(dcgain(G))
% % Get Q
% Q = G*D
% % Approx for Interaction
% Q_P = tf(pade(Q,6))
% % Store for reduction
% Q_R = Q_P;
% % Get Interaction
% k12 = Q_P(1,2).Numerator{:,:}(end-1) / Q_P(1,2).Denominator{:,:}(end)
% k21 = Q_P(2,1).Numerator{:,:}(end-1) / Q_P(2,1).Denominator{:,:}(end)
% % Calculate maximal kI
% ki2_m = abs(0.2/2/k12)
% ki1_m = abs(0.2/2/k21)
% 
% % Reduce the model to order -> use the time constants
% % HOW!? PAPER DOESN'T MENTION THE USED REDUCTION
% % Use FOTD Reduction Algorithm
% % Q11
% G1 = D(1,1)*G(1,1);
% G2 = D(2,1)*G(1,2);
% % Get the Coefficients
% K1 = dcgain(G1);
% T1 = G1.Denominator{:,:}(end-1)/G1.Denominator{:,:}(end);
% L1 = G1.IODelay;
% 
% K2 = dcgain(G2);
% T2 = G2.Denominator{:,:}(end-1)/G2.Denominator{:,:}(end);
% L2 = G2.IODelay;
% 
% % Set new parameter
% L = max(L1,L2);
% K = K1+K2;
% T = (K1*(T1+L1)+K2*(T2+L2))/K-L;
% 
% % Form new transfer function of the main Diagonal
% Q_R(1,1) = tf(K,[T,1],'IODelay',L);
% 
% % Q22
% G1 = D(1,2)*G(2,1);
% G2 = D(2,2)*G(2,2);
% % Get the Coefficients
% K1 = dcgain(G1);
% T1 = G1.Denominator{:,:}(end-1)/G1.Denominator{:,:}(end);
% L1 = G1.IODelay;
% 
% K2 = dcgain(G2);
% T2 = G2.Denominator{:,:}(end-1)/G2.Denominator{:,:}(end);
% L2 = G2.IODelay;
% 
% % Set new parameter
% L = max(L1,L2);
% K = K1+K2;
% T = (K1*(T1+L1)+K2*(T2+L2))/K-L;
% 
% % Form new transfer function of the main Diagonal
% Q_R(2,2) = tf(K,[T,1],'IODelay',L);
% 
% % Plot the systems
% figure(1)
% step(Q)
% hold on
% grid on
% step(Q_P)
% step(Q_R)
% legend('Original','Pade','FOTD Reduction')
% 
% 
% % Use the ki to calculate crossover freq
% w01 = sqrt(ki1_m/(Q_R(1,1).Denominator{:,:}(end-1)/Q_R(1,1).Denominator{:,:}(end)))
% w02 = sqrt(ki2_m/(Q_R(2,2).Denominator{:,:}(end-1)/Q_R(2,2).Denominator{:,:}(end)))
% 
% % Use crossover freq to calculate gain via damping
% zeta = 1/sqrt(2);
% kp1 = 2*zeta*w01-1
% kp2 = 2*zeta*w02-1
% 
% % Get AMIGO TUNED CONTROLLER
% C1 = AMIGO_Tune(Q_R(1,1),'PI');
% [kp1a,ki1a] = piddata(C1)
% 
% C2 = AMIGO_Tune(Q_R(2,2),'PI');
% [kp2a, ki2a] = piddata(C2)
% 
% % Detune Controller
% counter = 0
% while ki1a > ki1_m
%     if counter > 1e4
%         break
%     end
%     C1 = AMIGO_Detune(C1,Q_R(1,1));
%     [kp1a,ki1a] = piddata(C1);
%     counter = counter+1;
% end
% counter
% kp1a,ki1a
% % Detune 2
% counter = 0
% while ki2a > ki2_m
%     if counter > 1e5
%         break
%     end
%     C2 = AMIGO_Detune(C2,Q_R(2,2));
%     [kp2a,ki2a] = piddata(C2);
%     counter = counter+1;
% end
% counter
% kp2a,ki2a
% 
% 
% KI = [ki1_m,0;0,ki2_m]
% KI = D*KI
% KP = [kp1,0;0,kp2]
% KP = D*KP
% 
% C = pid2(KP,KI,0,0,0,0)
% % Close the loop
% 
% C = tf(C); % Convert to TF
% CA = C(1); % Set Point Controller
% CB = C(2); % Feedback Controller
% for Inputs = 1:2
%     for Outputs = 1:2
%         CR(Inputs,Outputs) = CA(:,:,Inputs,Outputs); % w -> u
%         CY(Inputs,Outputs) = CB(:,:,Inputs,Outputs); % y -> u
%     end
% end
% % Closed Loop 
% CL4 = CR*feedback(G,CY,+1);
%% Get Results
figure()
step(CL1)
hold on
grid on
step(CL2)
step(CL3)
legend('RGA','Decoupling with Q Design','Decoupling with G Design')
%% Get the stepinfo
inf1 = stepinfo(CL1)
inf2 = stepinfo(CL2)
inf3 = stepinfo(CL3)