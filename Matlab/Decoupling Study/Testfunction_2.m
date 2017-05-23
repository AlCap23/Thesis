%% Testcase 2 - Normalized Time
%  Random normalized Time between 0 and 1 for major and minor Diagonal
%  Save Ratio

%% Preprocessing
clear all
close all
clc

%% For Loop for Samples
% Sample size
samples = 2;
% Store the Coupling Factor for static system -> See Lunze II, p.194, Eq. 4.82
% Store the step Information
Info(samples) = struct('Ratio',1,'RGA',1,'Q',1,'G',1);


for sample = 1:1:samples+1
    %% Initialize the TF
    % Get the gainfactor
    norm1 = 0.99*rand(1,1)+0.01; %Random normalized time 1
    norm2 = 0.99*rand(1,1)+0.01; %Random normalized time 2
    
    % Make a random main diagonal gain with 20 and +-10
    k =  rand(2,2)*20 + 10*ones(2,1);
    T = rand(2,2)*50 + 150*ones(2,2);
    G = tf({k(1,1),k(1,2);0,k(2,2)},...
        {[T(1,1),1],[T(1,2),1];[T(2,1),1],[T(2,2),1]},...
        'IODelay',[norm1/(1-norm1)*T(1,1),norm2/(1-norm2)*T(1,2);0,0],...
        'InputName',{'Fan';'Valve'},...
        'OutputName',{'Temperature','Pressure'});
    
    % Get the determinant Information
    Info(sample).Ratio = norm1/norm2;
    
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
    CL2 = feedback(G,CY,+1)*CR;    
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
    CL3 = feedback(G,CY,+1)*CR;  
    %% Get the information
    Info(sample).RGA = stepinfo(CL1);
    Info(sample).Q = stepinfo(CL2);
    Info(sample).G = stepinfo(CL3);
end