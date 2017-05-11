%% Preprocess the Data extracted from Simulation
% Hygiene
clear all
close all
clc

% Read Excel
filename = 'KTL_50000.0_10_5.xlsx';
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
%% Analyse the Data
% make a colormap
c = linspace(TFData.OperatingPoint(1),TFData.OperatingPoint(end),length(TFData.OperatingPoint));

figure()
scatter(TFData.Fan_Temperature(1,:)./(TFData.Valve_Temperature(1,:)),TFData.Valve_Temperature(1,:)./(TFData.Fan_Temperature(1,:)),[],c,'filled')
xlabel('Fan Temperature / Valve Temperature')
ylabel('Valve Temperature / Fan Temperature')
title('Static Gains influencing Temperature')
colorbar
grid on
% xmax = max(max(TFData.Fan_Temperature(1,:)),max(TFData.Valve_Temperature(1,:)));
% xmin = min(min(TFData.Fan_Temperature(1,:)),min(TFData.Valve_Temperature(1,:)));
% axis([xmin xmax xmin xmax])

figure()
scatter(TFData.Fan_Pressure(1,:)./(TFData.Valve_Pressure(1,:)),TFData.Valve_Pressure(1,:)./(TFData.Fan_Pressure(1,:)),[],c,'filled')
xlabel('Fan Pressure / Valve Pressure')
ylabel('Valve Pressure / Fan Pressure')
title('Static Gains influencing Pressure')
colorbar
grid on

%% Get the interaction for every operation point
DK = TFData.Fan_Temperature(1,:).* TFData.Valve_Pressure(1,:) - TFData.Fan_Pressure(1,:).* TFData.Valve_Temperature(1,:);
kTp = TFData.Fan_Temperature(1,:).*TFData.Valve_Temperature(1,:) .*( TFData.Fan_Temperature(2,:)+TFData.Fan_Temperature(3,:) -  TFData.Valve_Temperature(2,:) - TFData.Valve_Temperature(3,:))  
kpT = TFData.Valve_Pressure(1,:).*TFData.Fan_Pressure(1,:) .*( -TFData.Fan_Pressure(2,:)-TFData.Fan_Pressure(3,:) +  TFData.Valve_Pressure(2,:) + TFData.Valve_Pressure(3,:))  

figure()
plot(TFData.OperatingPoint,DK./(TFData.Fan_Temperature(1,:).* TFData.Valve_Pressure(1,:)),'o-')
xlabel('Operating Point')
ylabel('Determinant of the Static Gain')
title('Coupling Estimation via the Determinant')
grid on

figure()
scatter(kTp,kpT,[],c,'filled')
xlabel('Coupling Factor from Pressure to Temperature')
ylabel('Coupling Factor from Temperature to Pressure')
title('Coupling Factors')
colorbar
grid on

%% Other Analysis Tools
% Static Gains
% figure()
% scatter(TFData.Fan_Temperature(1,:),TFData.Valve_Temperature(1,:),[],c,'filled')
% xlabel('Fan Temperature')
% ylabel('Valve Temperature')
% title('Static Gains influencing Temperature')
% colorbar
% grid on
% xmax = max(max(TFData.Fan_Temperature(1,:)),max(TFData.Valve_Temperature(1,:)));
% xmin = min(min(TFData.Fan_Temperature(1,:)),min(TFData.Valve_Temperature(1,:)));
% axis([xmin xmax xmin xmax])
% 
% figure()
% scatter(TFData.Fan_Pressure(1,:),TFData.Valve_Pressure(1,:),[],c,'filled')
% xlabel('Fan Pressure')
% ylabel('Valve Pressure')
% title('Static Gains influencing Pressure')
% colorbar
% grid on
% xmax = max(max(TFData.Fan_Pressure(1,:)),max(TFData.Valve_Pressure(1,:)));
% xmin = min(min(TFData.Fan_Pressure(1,:)),min(TFData.Valve_Pressure(1,:)));
% axis([xmin xmax xmin xmax])
% 
% figure()
% plot(TFData.OperatingPoint,abs(TFData.Fan_Temperature(1,:)./TFData.Valve_Temperature(1,:)),'o-')
% hold on
% plot(TFData.OperatingPoint,abs(TFData.Valve_Pressure(1,:)./TFData.Fan_Pressure(1,:)),'o-')
% xlabel('Operating Point')
% ylabel('Relative Influence')
% title('Relative Influence of the Main Couplings')
% grid on
% 
% figure()
% scatter(abs(TFData.Fan_Temperature(1,:)./TFData.Valve_Temperature(1,:)),abs(TFData.Valve_Pressure(1,:)./TFData.Fan_Pressure(1,:)),[],c,'filled')
% xlabel('Fan Temperature')
% ylabel('Valve Pressure')
% title('Relative Influence of the Main Couplings')
% colorbar
% grid on
% %xmax = max(max(TFData.Fan_Pressure(1,:)),max(TFData.Valve_Pressure(1,:)));
% %xmin = min(min(TFData.Fan_Pressure(1,:)),min(TFData.Valve_Pressure(1,:)));
% %axis([xmin xmax xmin xmax])
% 
% 
% % Time Constants
% figure()
% scatter(TFData.Fan_Temperature(2,:),TFData.Valve_Temperature(2,:),[],c,'filled')
% xlabel('Fan Temperature')
% ylabel('Valve Temperature')
% title('Time Constant influencing Temperature')
% colorbar
% grid on
% xmax = max(max(TFData.Fan_Temperature(2,:)),max(TFData.Valve_Temperature(2,:)));
% xmin = min(min(TFData.Fan_Temperature(2,:)),min(TFData.Valve_Temperature(2,:)));
% axis([xmin xmax xmin xmax])
% 
% figure()
% scatter(TFData.Fan_Pressure(2,:),TFData.Valve_Pressure(2,:),[],c,'filled')
% xlabel('Fan Pressure')
% ylabel('Valve Pressure')
% title('Time Constant influencing Pressure')
% colorbar
% grid on
% xmax = max(max(TFData.Fan_Pressure(2,:)),max(TFData.Valve_Pressure(2,:)));
% xmin = min(min(TFData.Fan_Pressure(2,:)),min(TFData.Valve_Pressure(2,:)));
% axis([xmin xmax xmin xmax])
% 
% 
% % Delay
% figure()
% scatter(TFData.Fan_Temperature(3,:),TFData.Valve_Temperature(3,:),[],c,'filled')
% xlabel('Fan Temperature')
% ylabel('Valve Temperature')
% title('Delay influencing Temperature')
% colorbar
% grid on
% xmax = max(max(TFData.Fan_Temperature(3,:)),max(TFData.Valve_Temperature(3,:)));
% xmin = min(min(TFData.Fan_Temperature(3,:)),min(TFData.Valve_Temperature(3,:)));
% axis([xmin xmax xmin xmax])
% 
% figure()
% scatter(TFData.Fan_Pressure(3,:),TFData.Valve_Pressure(3,:),[],c,'filled')
% xlabel('Fan Pressure')
% ylabel('Valve Pressure')
% title('Delay Constant influencing Pressure')
% colorbar
% grid on
% xmax = max(max(TFData.Fan_Pressure(3,:)),max(TFData.Valve_Pressure(3,:)));
% xmin = min(min(TFData.Fan_Pressure(3,:)),min(TFData.Valve_Pressure(3,:)));
% axis([xmin xmax xmin xmax])



