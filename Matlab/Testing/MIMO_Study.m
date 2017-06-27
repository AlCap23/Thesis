%% MIMO Study
% Study to investigate the Maximum Singular Value of the AMIGO Controller
% with respect to systemorder
clear all
close all
clc

%% Outer Loop
% Set maximum Order and maximum Sample Size
maxOrder = 10;
maxSamples = 10;
% Make a Loop for System Order 1-10
input = 2;
output = 2;
% Preallocate Data
MS = zeros(maxOrder,maxSamples,2);

for order = 1:1:maxOrder
    for samples = 1:1:maxSamples
        % Make a Random System
        G = rtf(order,input,output);
        % Simulate experiment
        opt = stepDataOptions('InputOffset',0,'StepAmplitude',1);
        [y,t] = step(G,opt);
        u = ones(size(t));
        % Predefine the FOTD Model
        G_A = tf(ones(input,output));
        C_A = tf(ones(input,output));
        % Outer Loop
        for inputs = 1:input
            for outputs = 1:output
                G_A(inputs,outputs) = FOTD_AREA(y(:,inputs,outputs),u,t);
                % Create Controller
                C_A(inputs,outputs) = AMIGO_Tune(G_A(inputs,outputs),'PI');
            end
        end
        % Get the Maximum Singular Value
        [sv,wv] = sigma(eye(inputs,outputs)+G*C_A,[],1);
        [svmax, locmax] = max(sv(1,:));
        wmax = wv(locmax);
        % Store the Data
        MS(order,samples,1) = svmax;
        % Get the Maximum Singular Value
        [sv,wv] = sigma(eye(inputs,outputs)+G_A*C_A,[],1);
        [svmax, locmax] = max(sv(1,:));
        wmax = wv(locmax);
        % Store the Data
        MS(order,samples,2) = svmax; 
    end
end