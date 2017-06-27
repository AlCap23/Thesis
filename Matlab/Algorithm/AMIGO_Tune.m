function C = AMIGO_Tune(FOTD, TYPE)
%Computes a controller based on the AMIGO Rules by Astr�m ,
%H�gglund, Adavanced PID Control, p.227 ff
%The function returns a Controller of PI or PID type. The controller is
%tuned by using the AMIGO principles described by Astr�m and H�gglund in
%'Advanced PID Control' p. 227 ff. 
%
%The Controllers are derived by using rules based on several test
%batches - which are approximated by a First Order Time Delay (FOTD)
%Model - controlled by PI or PID controllers which are optimized. 

%% Inputs
%  FOTD - A First Order Time Delay Model of the Process
%  Type - String for determination of the controller. Either 'PI' or 'PID'

%% Outputs
%  C - The resulting Controller of the form
%
%   C = Kp*(1 + 1/Ti * 1/s + Td * s ) 
%% Get Parameter

% Check System Order
if order(FOTD) ~=1
    disp('System is of higher Order! First Order Time Delay Model is requiered!')
    disp('Reduced Model is used')
    % Approximate a System with Relay_Identification
    %FOTD = Relay_Identification(FOTD);
    FOTD = tf(reduce(ss(FOTD),1));
end

% Get System Parameter
K_P = dcgain(FOTD);
T = FOTD.Denominator{:,:}(end-1)/FOTD.Denominator{:,:}(end);

if FOTD.OutputDelay >0
    L = FOTD.OutputDelay;
elseif FOTD.InputDelay >0
    L = FOTD.InputDelay;
elseif FOTD.IODelay >0
    L = FOTD.IODelay;
else
    L = 0;
end

% If the Output Delay is smaller than 1e-2 use 1e-2 instead
if L < 0.2*T 
    if 0.2*T < 1e-2
        L = 1e-2;
    else
        L = 0.2*T;
    end
    
    disp('Output Delay is zero or small! Algorithm uses L = 0.2*T instead!')
end


% Check if Type is given and set PI as Standard
if ~exist('TYPE','var')
    TYPE = 'PI';
end

switch TYPE
    % PI Tuning Rules
    case 'PI'
        % Proportional gain of the controller
        K = 0.15/ K_P + (0.35 - (L*T)/(L+T)^2)*T/K_P/L ; % Eq. 7.5 or Eq. 7.32, checked
        % Integral gain of the controller
        T_i = 0.35*L + 13*L*T^2 /(T^2+12*L*T+7*L^2); % Eq. 7.5 or Eq. 7.32, checked
        % Make a PI controller
        C = pidstd(K,T_i);
        
    % PID Tuning Rules
    case 'PID'
        
        if L/(T+L) - 0.3 < 1e-5 % Constrain given on p.263 f.
            disp('Process is not optimal for use of AMIGO PID Rules!')
        end
        
        % Proportional gain of the controller
        K = 1/K_P * ( 0.2+0.45*T/L); % Eq. 7.7 or Eq. 7.33, checked
        % Integral gain of the controller
        T_i = ( 0.4*L + 0.8*T )*L /(L + 0.1*T); % Eq. 7.7 or Eq. 7.33, checked
        % Derivative gain of the controller
        T_D = 0.5*L*T / ( 0.3*L + T); % Eq. 7.7 or Eq. 7.33, checked
        % Make a PID controller
        C = pidstd(K,T_i,T_D);
    otherwise
        error('Choose controller type! Accepted Inputs are PI or PID')       
        
end

