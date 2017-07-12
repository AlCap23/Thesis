%% MIMO Study
% Study to investigate the Maximum Singular Value of the AMIGO Controller
% with respect to systemorder
clear all
close all
clc

%% Outer Loop
% Set maximum Order and maximum Sample Size
maxOrder = 10;
maxSamples = 20;
% Make a Loop for System Order 1-10
input = 2;
output = 2;
% Constrains for Controller
Constrains = [0.01,0.2,sqrt(2),sqrt(2)];
% Preallocate Data
SV = zeros(maxOrder,maxSamples,3);

for order = 1:1:maxOrder
    for samples = 1:1:maxSamples
        order,samples
        %% Make a Random System and Identify
        G = rtf(order,input,output);
        % Make a Model
        G_M = tf(zeros(input,output));
        % Simulate experiment
        opt = stepDataOptions('InputOffset',0,'StepAmplitude',1);
        % Loop throught inputs & outputs
        for inputs = 1:1:input
            for outputs = 1:1:output
                GCurrent = G(inputs,outputs);
                % Experiment
                [y,t] = step(GCurrent,opt);
                u = ones(size(t));
                % Identification
                G_M(inputs,outputs) = FOTD_AREA(y,u,t);
            end
        end
        
        %% Make a Controller RGA
        C1 = Decoupling_RGA(G_M,Constrains,'DC',0);
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
        %% Store the Data
        % Get the Maximum Singular Value
        [sv,wv] = sigma(S1,[],1);
        [svmax, locmax] = max(sv(1,:));
        wmax = wv(locmax);
        % Store the Data
        SV(order,samples,1) = svmax;
        %% Make Controller via Astroem Algorithm
        C2 = Decoupling_A(G_M,Constrains,'AMIGO',0);
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
        %% Store the Data
        % Get the Maximum Singular Value
        [sv,wv] = sigma(S2,[],1);
        [svmax, locmax] = max(sv(1,:));
        wmax = wv(locmax);
        % Store the Data
        SV(order,samples,2) = svmax;
        %% Make Controller via Modified Astroem
        C3 = Decoupling_F(G_M,Constrains,'AMIGO',0);
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
        %% Store the Data
        % Get the Maximum Singular Value
        [sv,wv] = sigma(S3,[],1);
        [svmax, locmax] = max(sv(1,:));
        wmax = wv(locmax);
        % Store the Data
        SV(order,samples,3) = svmax;
%         %% Check via Plotting
%         figure(1)
%         step(CL1)
%         hold on
%         grid on
%         step(CL2)
%         step(CL3)
%         legend('RGA','Decoupling with Q Design','Decoupling with G Design')
%         clf
%         %% Store Data
%         % Get the Maximum Singular Value
%         [sv,wv] = sigma(eye(inputs,outputs)+G*C_A,[],1);
%         [svmax, locmax] = max(sv(1,:));
%         wmax = wv(locmax);
%         % Store the Data
%         MS(order,samples,1) = svmax;
%         % Get the Maximum Singular Value
%         [sv,wv] = sigma(eye(inputs,outputs)+G_A*C_A,[],1);
%         [svmax, locmax] = max(sv(1,:));
%         wmax = wv(locmax);
%         % Store the Data
%         MS(order,samples,2) = svmax; 
    end
end