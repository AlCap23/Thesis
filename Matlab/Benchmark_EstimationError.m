%% Benchmark Model Error 
% Create Second Order Systems and approximate them with First Order Models

clear all
close all
clc

%% Loop for n cycles and get random Processes

n = 50000; % Sample number
time = 0:1:10000; % Time to simulate
u = ones(length(time),1); % Input , Step
Model_Error = zeros(n,1); % Preallocate
PM = 2*asin(1/2/sqrt(2)); % Phase Margin

for loop = 1:1:n
    % Create a Model with two Random Time Constants
    T = (randn(2)*sqrt(40))+200*ones(2,1);
    % Create a Gain
    K = randn(1)*sqrt(9.0)+10;
    % Create a System
    P = tf(K, [T(1),1])*tf(1,[T(2),1]);
    % Approximate the system via sum of the time constants
    P_Mod = tf(K,[sum(T),1]);
    % Create a Controller with a selected phase margin
    opts = pidtuneOptions('PhaseMargin',PM);
    C = pidtune(P_Mod,'PI',opts);
    % Make the closed Loop
    CL_Mod = feedback(P_Mod*C,1);
    % Get the error via the relative error in the phase margin
    [gm,pm,wgm,wpm] = margin(CL_Mod);
    Model_Error(loop) = pm/PM;
end
