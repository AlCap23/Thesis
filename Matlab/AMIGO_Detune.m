function c = AMIGO_Detune(C,TF,k_p,MS)
%Detunes a controller based on the detuning rules
%   Detailed explanation goes here


%% Get the parameters of the Controller
[K_P,K_I,K_D] = piddata(C);

%% Get the parameters of the Process

% Get the Static Gain of the Process
K = dcgain(TF);
T = TF.Denominator{:,:}(end-1) / TF.Denominator{:,:}(end);
L = TF.OutputDelay;
tau = L / (L+T);

%% Detune a PI Controller based on MS
% Compute a new proportional gain if not given
if ~exist('k_p','var')
    k_p = 0.95*K_P;
end
% Maximum Sensitivity if not given
if ~exist('MS','var')
    MS = sqrt(2);
end

beta = MS*(MS+sqrt(MS^2-1))/2;
alpha = (MS-1)/MS;

% Test if controller is PI Controller
if K_D == 0
    if tau > 0.1
        k_i = K_I* (K*k_p+alpha)/(K*K_P+alpha);
    else
        % Test for Condition
        if K*k_p - K_I*k_p*(L+T) / (beta*(alpha+K_P*K)) - alpha > 1e-10
            k_i = K_I * (alpha+K*k_p)/(alpha+K*K_P);
        else
            k_i = beta * (alpha+K*k_p)^2/(K*(L+T));
        end
    end
    % Make a new controller
    c = pid(k_p,k_i,0);
else
    error('PID Detuning is not implemented! Use a PI Tuning approach')
end


end

