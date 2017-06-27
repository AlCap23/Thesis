%% SISO Study

clear all
close all
clc

%% Outer Loop
% Make a Loop for System Order 1-10
for order = 1:1:10
    for samples = 1:1:100
        % Make a Random System
        G = rss(order,1,1)
        % Simulate experiment
        opt = stepDataOptions('InputOffset',0,'StepAmplitude',1);
        [y,t] = step(G,opt);
        u = ones(size(y));
        % Approximate with FOTD AREA
        G_A = FOTD_AREA(y,u,t);
        % Create Controller
        C_A = AMIGO_Tune(G_A,'PI');
        % Create Sensitivity of the Real System
        S = 1/(1+G*C_A);
        % Get the Maximum Sensitivity
        MS_R = getPeakGain(S)
        % Create Sensitivity of the Identified System
        S = pade(1/(1+G_A*C_A),4);
        % Get the Maximum Sensitivity
        MS_A = getPeakGain(S)
        
        % Approximate with Relay
        %G_R = Relay_Identification(tf(G));
        % Create Controller
        %C_R = AMIGO_Tune(G_R,'PI');
        
    end
end