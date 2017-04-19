%% Testing the AMIGO Rules based on FOTD

% Hygiene
clear all
close all
clc

%% Initialize

% System - FOTD
G = tf([10],[10,1],'OutputDelay',9);

% Get System Parameter
K_P = G.Numerator{:,:}(end);
T = G.Denominator{:,:}(end-1);
L = G.OutputDelay;

%% PI Tuning
% Based on FOTD
K = 0.15/ K_P + (0.35- (L*T)/(L+T)^2)*T/K_P/L ;
T_i = 0.35*L + 13*L*T^2 /(T^2+12*L*T+7*L^2);

C = pidstd(K,T_i);

% Simulation

OL_PI = C*G;

%% PID Tuning
% Based on FOTD
K = 1/K_P * ( 0.2+0.45*T/L);
T_i = ( 0.4*L + 0.8*T )*L /(L + 0.1*T);
T_D = 0.5*L*T / ( 0.3*L + T);

C = pidstd(K,T_i,T_D);

OL_PID = C*G;


