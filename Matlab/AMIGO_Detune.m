function c = AMIGO_Detune(C,TF,k_p,MS)
%Computes a controller based on the AMIGO Detuning Rules by Aström ,
%Hägglund, Adavanced PID Control, p.253 ff
%The function returns a Controller of PI or PID type. The controller is
%tuned by using the AMIGO principles described by Aström and Hägglund in
%'Advanced PID Control' p. 253 ff. 
%
%The Controllers are derived by using rules based on several test
%batches - which are approximated by a First Order Time Delay (FOTD)
%Model - controlled by PI or PID controllers which are optimized. 

%% IMPORTANT NOTE
%  Parameter in CAPITAL letter are initial parameter
%  Parameter in small letters are new, detuned parameter
%% Get the parameter of the controller
[K_P,K_I,K_D] = piddata(C);

%% Get the parameter of the Process

% Get the Static Gain of the Process
K = dcgain(TF);
T = TF.Denominator{:,:}(end-1) / TF.Denominator{:,:}(end);
L = TF.OutputDelay;
tau = L / (L+T);

%% Detune a PI Controller based on MS
% Compute a new proportional gain if not given
if ~exist('k_p','var')
    k_p = 0.8*K_P;
end

% Maximum Sensitivity if not given
if ~exist('MS','var')
    MS = sqrt(2);
end

% Compute coefficients
beta = MS*(MS+sqrt(MS^2-1))/2; % Eq. 7.24, checked
alpha = (MS-1)/MS; % Eq. 7.19, checked

% Test if controller is PI Controller
if K_D == 0
    % Use normalized time to determine Process as explained on p.255 f.
    if tau > 0.1
        k_i = K_I* (K*k_p+alpha)/(K*K_P+alpha); % Eq 7.20, checked
    else
        % Test for Condition as given in Eq. 7.27
        if K*k_p - K_I*K*(L+T) / (beta*(alpha+K_P*K)) - alpha > 1e-10
            k_i = K_I * (alpha+K*k_p)/(alpha+K*K_P); % Eq. 7.27, checked
        else
            k_i = beta * (alpha+K*k_p)^2/(K*(L+T)); % Eq. 7.27, checked
        end
    end
    % Make a new controller
    c = pid(k_p,k_i,0);
else
    error('PID Detuning is not implemented! Use a PI Tuning approach')
end


end

