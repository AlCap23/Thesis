function C = AMIGO_Tune(FOTD, TYPE)
%AMIGO Computes a controller based on the AMIGO Rules by Aström ,
%Hägglund, Adavanced PID Control, p.227 ff
%   The function returns a Controller of PI or PID type. The controller is
%   tuned by using the AMIGO principles described by Aström and Hägglund in
%   'Advanced PID Control' p. 227 ff. 
%   The Controllers are derived by using rules based on several test
%   batches - which are approximated by a First Order Time Delay (FOTD)
%   Model - controlled by PI or PID controllers which are optimized. 

%% Inputs
%  FOTD - A First Order Time Delay Model of the Process
%  Type - String for determination of the controller. Either 'PI' or 'PID'

%% Outputs
%  C - The resulting Controller of the form
%
%   C = Kp*(1 + 1/Ti * 1/s + Td * s )
%
  
%% Get Parameters

% Check System Order
if order(FOTD) ~=1
    error('System is of higher Order! First Order Time Delay Model is requiered!')
end

% Get System Parameter
K_P = FOTD.Numerator{:,:}(end);
T = FOTD.Denominator{:,:}(end-1);
L = FOTD.OutputDelay;

% If the Output Delay is smaller than 0.2*T use 0.2*T instead
if L < 0.2*T
    L = 0.2*T;
    disp('Output Delay is zero or small! Algorithm uses 0.2*T instead!')
end

switch TYPE
    % PI Tuning Rules
    case 'PI'
        % Proportional gain of the controller
        K = 0.15/ K_P + (0.35- (L*T)/(L+T)^2)*T/K_P/L ;
        % Integral gain of the controller
        T_i = 0.35*L + 13*L*T^2 /(T^2+12*L*T+7*L^2);
        % Make a PI controller
        C = pidstd(K,T_i);
        
    % PID Tuning Rules
    case 'PID'
        % Proportional gain of the controller
        K = 1/K_P * ( 0.2+0.45*T/L);
        % Integral gain of the controller
        T_i = ( 0.4*L + 0.8*T )*L /(L + 0.1*T);
        % Derivative gain of the controller
        T_D = 0.5*L*T / ( 0.3*L + T);
        % Make a PID controller
        C = pidstd(K,T_i,T_D);
    otherwise
        error('Choose controller type! Accepted Inputs are PI or PID')       
        
end

