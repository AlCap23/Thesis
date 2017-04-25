function [ G_M, gamma] = Relay_Identification(G)
%Approximates a given system G by a First Order Time Delay (FOTD) Model.
%The Process is described in "Asymmetric relay autotuning - Practical
%features for industrial use" by Berner, Hägglund, Aström, DOI http://dx.doi.org/10.1016/j.conengprac.2016.05.017
%% Inputs
%   G - System to be approximated
%% Outputs
%   G_M - FOTD Model of the System
%% Initialize Variables for Experiment
% Simulation
t_start = 0; % Start Time in s
t_end = 6000; % Stop Time in s, High Order require more Time
dt = 0.01; % Time Step in s

% Relay Parameters
u0 = 10 ; % Constant Input on the Plant
y0 = dcgain(G)*u0; % Set Point / Working Point
mu = 10; % Parameter for estimating hysteresis -> High Value is better for Benchmark
d1 = 10; % Upper Limit of the Output
d2 = 2; % Lower Limit of the Output
h = min(d1,d2)*abs(dcgain(G))/mu; % Hysterisis for Relay, from Eq. 19
L = G.OutputDelay; % Time Delay

% Set a small delay if none is given ( for stability of the simulation )
if L < 0.5*G.Denominator{:,:}(end-1)
    L = 0.5*G.Denominator{:,:}(end-1);
end
[Num, Den] = tfdata(G,'v'); % Get the Transfer Function Data
%% Simulink

% Load the system
load_system('Relay_Auto_Tuning.slx');
    
% Simulate the system IN THE CURRENT WORKSPACE
res =  sim('Relay_Auto_Tuning.slx','SrcWorkspace','current');
y = res.y; % Output of the Plant as timeseries
u = res.u; % Input of the Plant as timeseries

%plot(y)
%% Get the Plant Parameter
% Take sample from the middle of the Test
time_limit = round(length(y.Time)/10) ; % Assume half the time is sufficient 
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
I_y = sum(y_Data(ypeakloc(1):ypeakloc(2))-y0)*sample_rate; % Riemann Sum as Integration
% Get the integral action of the input
I_u = (upeaks(1)-u0)*t_on + (u_Min-u0)*t_off; % Eq. 12, checked
% Static Gain
K_P = I_y/I_u; % Eq. 13, checked
% Calculate the normalized time , Time Constant and Delay
rho = max(t_on,t_off)/min(t_on,t_off); % Half Period as given in Eq. 10, checked
gamma = max(d1,d2) / min(d1,d2); % Asymetry level, Eq. 7, checked
tau = (gamma-rho) / (gamma-1) / (0.35*rho + 0.65); % Normalized Time, Eq.9, checked

% Time Constant
T_on = t_on / log( ((h/abs(K_P))-d2+exp(tau/(1-tau))*(d1+d2)) / (d1- (h/abs(K_P)) ) ); % Eq. 14, checked
T_off = t_off / log( ((h/abs(K_P))-d1+exp(tau/(1-tau))*(d1+d2)) / (d2- (h/abs(K_P)) ) ); % Eq. 15, checked
T = 1/2*(T_on+T_off); % Get mean of the Time Constants to make error from t_on / t_off smaller
L = T* (tau/(1-tau)); % Eq.16, checked


%% Make FOTD Model
%K_P,T,L, tau, rho, gamma
G_M = tf(K_P,[T,1],'OutputDelay',L);

end

