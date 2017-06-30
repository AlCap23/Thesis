%% SISO Study
% Study to investigate the Maximum Sensitivity of the AMIGO Controller
% with respect to systemorder
clear all
close all
clc

%% Outer Loop
% Set maximum Order and maximum Sample Size
maxOrder = 10;
maxSamples = 100;
% Preallocate Data
MS = zeros(maxOrder,maxSamples,2);

% Make a Loop
for order = 1:1:maxOrder
    for samples = 1:1:maxSamples
        % Make a Random System
        G = rtf(order,1,1);
        % Simulate experiment
        opt = stepDataOptions('InputOffset',0,'StepAmplitude',1);
        [y,t] = step(G,opt);
        u = ones(size(y));
        % Approximate with FOTD AREA
        %G_A = FOTD_AREA(y,u,t);
        % Approximate with Relay Autotuning
        G_A = Relay_Identification(G);
        % Create Controller
        C_A = AMIGO_Tune(G_A,'PI');
        % Create Sensitivity of the Real System
        S = pade(1/(1+G*C_A),4);
        % Get the Maximum Sensitivity
        % Store the Data
        MS(order,samples,1) = getPeakGain(S);
        % Create Sensitivity of the Identified System
        S = pade(1/(1+G_A*C_A),4);
        % Get the Maximum Sensitivity
        % Store the Data
        MS(order,samples,2) = getPeakGain(S);
    end
end