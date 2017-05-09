%% Preprocess the Data extracted from Simulation
% Hygiene
clear all
close all
clc

% Read Excel
filename = 'KTL_80000.0_10_5.xlsx';
% Import FOTD Parameter
Data11 = xlsread(filename,'Fan_Temperature','B2:U4');
Data21 = xlsread(filename,'Fan_Pressure','B2:U4');
Data12 = xlsread(filename,'Valve_Temperature','B2:U4');
Data22 = xlsread(filename,'Valve_Pressure','B2:U4');
Points = xlsread(filename,'Fan_Temperature','B1:U1');
% Normalize the Gains
s_FT = 1;       % Fan to Temperature
s_VT = 1/1e6;   % Valve to Temperature
s_FP = 1/1e5;   % Fan to Pressure
s_VP = 1/1e11;  % Valve to Pressure
Data11(1,:) = s_FT*Data11(1,:);
Data12(1,:) = s_VT*Data12(1,:);
Data21(1,:) = s_FP*Data21(1,:);
Data22(1,:) = s_VP*Data22(1,:);
% Create a structure
TFData = struct('OperatingPoint',Points,...
                'Fan_Temperature',Data11,...
                'Valve_Temperature',Data12,...
                'Fan_Pressure',Data21,...
                'Valve_Pressure',Data22);
% Store the struct
save('KTL80000.mat','TFData')