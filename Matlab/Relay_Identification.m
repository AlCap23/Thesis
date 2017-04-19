function [ G_M ] = Relay_Identification( G )
%Relay_Identification Approximates a given system G by a FOTD Model
%   Detailed explanation goes here

%% Initialize Variables for Experiment
% Simulation
t_start = 0; % Start Time in s
t_end = 1000; % Stop Time in s
dt = 0.1; % Time Step in s

% Plant Parameter
Num = G.Numerator{:,:};
Den = G.Denominator{:,:};
L = G.OutputDelay;

% Tuning Parameters
u0 = 20 ; % Constant Input on the Plant
y0 = dcgain(G)*u0; % Set Point / Working Point
h = 1; % Hysterisis for Relay
d1 = 10; % Upper Limit of the Output
d2 = 3; % Lower Limit of the Output

%% Simulink

load_system('Relay_Auto_Tuning.slx');
sim('Relay_Auto_Tuning.slx');

plot(y)
%% Get the Plant Parameter
% Take sample from the middle of the Test
time_limit = round(length(y.Time)/4) ; % Assume half the time is sufficient 
y_Data = y.Data(time_limit:end); % Get the Amplitude of the plant
u_Data = u.Data(time_limit:end); % Get the Input
time_Data = y.Time(time_limit:end); % Get the Time Sample
sample_rate = time_Data(2)-time_Data(1); % Assume constant sample rate


% Find the Peaks in the Data
[ypeaks, ypeakloc] = findpeaks(y_Data); % Get the location and the value of the peaks
[upeaks, upeakloc] = findpeaks(u_Data);

% Since we know that we got a oscillation, use that knowledge to find first
% minimum
[u_Min, loc_u_Min] = min(u_Data(upeakloc(1):upeakloc(2)));
loc_u_Min = loc_u_Min + upeakloc(1);

% Get the on time from first maximum to minimum of u
t_on = time_Data(loc_u_Min) - time_Data(upeakloc(1));
% Get the off time from minimum to second maximum
t_off = time_Data(upeakloc(2)) - time_Data(loc_u_Min);

% Get the numerical integral of the plant process output
I_y = sum(y_Data(ypeakloc(1):ypeakloc(2))-y0)*sample_rate;
% Get the integral action of the input
I_u = (upeaks(1)-u0)*t_on + (u_Min-u0)*t_off;
% Static Gain
K_P = I_y/I_u;
% Calculate the normalized time , Time Constant and Delay
rho = max(t_on,t_off)/min(t_on,t_off); % Half Period as given in Eq. 10
gamma = max(d1,d2) / min(d1,d2); % Asymetry level, Eq. 7
tau = (gamma-rho) / (gamma-1) / (0.35*rho + 0.65); %Normalized Time, Eq.9
% Time Constant
T_on = t_on / log( (h/abs(K_P)-d2+exp(tau/(1-tau))*(d1+d2)) / (d1-h/abs(K_P)) );
T_off = t_off / log( (h/abs(K_P)-d1+exp(tau/(1-tau))*(d1+d2)) / (d2-h/abs(K_P)) );
T = 1/2*(T_on+T_off);
L = T* (tau/(1-tau));

%% Make FOTD Model

G_M = tf(K_P,[T,1],'OutputDelay',L);

end

