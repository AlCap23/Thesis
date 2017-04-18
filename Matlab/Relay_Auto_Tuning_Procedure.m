%% Relay Auto Tuning Experiment

% Hygiene
clear all
close all
clc

%% Initialize Variables

% Simulation
t_start = 0; % Start Time in s
t_end = 200; % Stop Time in s

% Plant Parameter
Num = [10]; % Numerator of the Transfer Function
Den = [10,1]; % Denumerator of the Transfer Function
L = 9; % Time Delay of the Transfer Function

% Tuning Parameters
y0 = 10; % Set Point / Working Point
u0 = 2 ; % Constant Input on the Plant
h = 0.1; % Hysterisis for Relay
d1 = 5; % Upper Limit of the Output
d2 = 2; % Lower Limit of the Output

%% Simulink

load_system('Relay_Auto_Tuning.slx');
sim('Relay_Auto_Tuning.slx');

%% Postprocess Data

% figure()
% plot(u)
% hold on
% plot(y)
% grid on
% xlabel('Time [s]');
% ylabel('Input and Output');
% title('Relay Autotuning');

%% Get the Plant Parameter
% Take sample from the middle of the Test
time_limit = round(length(y.Time)/4) ; % Assume half the time is sufficient 
y_Data = y.Data(time_limit:end); % Get the Amplitude of the plant
u_Data = u.Data(time_limit:end); % Get the Input
time_Data = y.Time(time_limit:end); % Get the Time Sample
sample_rate = time_Data(2)-time_Data(1); % Assume constant sample rate


% Get the on and off time
lower_limit = round(length(y_Data)/4)
upper_limit =  round(length(y_Data)*3/4)
% Find first minimum of u and minimum of y before that
[y_min, pos_y_min] = min(y_Data( lower_limit : 2*lower_limit ) )
pos_y_min = pos_y_min+round(length(y_Data)/4)
[u_min, pos_u_min] = min(u_Data(pos_y_min+1:upper_limit))
pos_u_min = pos_u_min + pos_y_min
% Find first maximum of u and minimum of y since u_min and u_max
[y_max, pos_y_max] = max(y_Data(pos_y_min:upper_limit)); % This order is more stable
pos_y_max = pos_y_max + pos_y_min
[u_max, pos_u_max] = max(u_Data(pos_y_max:upper_limit));
pos_u_max = pos_u_max+pos_y_max
% Calculate off time
t_off = time_Data(pos_u_max)-time_Data(pos_u_min)
% Calculate the first half of the integral output
%I_u = sum(u_Data(pos_u_min:pos_u_max))*sample_rate;
I_y = sum(y_Data(pos_y_min:pos_y_max)-y0)*sample_rate;
% Get the next minimum
[y_min,pos_y_min] = min(y_Data(pos_u_max:upper_limit));
pos_y_min = pos_y_min+pos_u_max
[u_min, pos_u_min] = min(u_Data(pos_y_min:upper_limit));
pos_u_min = pos_u_min+pos_y_min
% Get the on time
t_on = time_Data(pos_u_min)-time_Data(pos_u_max)

% Calculate the second half of the integral output
%I_u = I_u + sum(u_Data(pos_u_max:pos_u_min))*sample_rate;
I_y = I_y + sum(y_Data(pos_y_max:pos_y_min)-y0)*sample_rate;
% Calculate the integral of the relay output
I_u = abs(d1)*t_on-abs(d2)*t_off ; % Eq. 12

% Calculate the normalized time , Time Constant and Delay
rho = max(t_on,t_off)/min(t_on,t_off); % Half Period as given in Eq. 10
gamma = max(d1,d2) / min(d1,d2); % Asymetry level, Eq. 7
tau = gamma-rho / (gamma-1) / (0.35*rho + 0.65); %Normalized Time, Eq.9

% Static Gain
K_P = I_y/I_u
% Time Constant
T_on = t_on / log( (h*abs(K_P)-d2+exp(tau/(1-tau))*(d1+d2)) / (d1-h*abs(K_P)) )
T_off = t_off / log( (h*abs(K_P)-d1+exp(tau/(1-tau))*(d1+d2)) / (d2-h*abs(K_P)) )