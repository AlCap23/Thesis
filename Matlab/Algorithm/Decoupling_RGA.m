function C = Decoupling_RGA(TF,Constrains,TYPE,SetPointWeight)
% Returns the controller
% Set Standard Type
if ~exist('TYPE','var')
    TYPE = 'DC';
end
% Check for given Set Point Weight
if ~exist('SetPointWeight','var')
    % If none is given, make set point weight off controller
    b = 0; 
else
    b = SetPointWeight;
end

% System Size
sys_size = size(TF);

% Constrains
if ~exist('Constrains','var')
    % Set MS to sqrt(2)
    Constrains = [sqrt(2), sqrt(2)];
end

MS = Constrains(1,1:2); % Maximum of the sensitivity
PM = 2*asin(1/2./MS); % Phase Margin is at least 2 arcsin(1/2/Ms)
MProd = prod(MS); % Sensitivy Product

% Make a RGA
switch TYPE
    case 'DC'
        RGA = dcgain(TF).*inv(dcgain(TF))';
    case 'Crossover'
        CW = Inf; % Initialize Crossover Frequency at Infinity
        % Loop over every Transfer Function
        for outputs = 1:sys_size(2)
            for inputs = 1:sys_size(1)
                % Get the stability margin and with it the crossover
                [Gm, Pm, Wgm, Wpm] = margin(TF(outputs,inputs));
                if Wpm < CW
                    % Set crossover
                    CW = Wpm
                end
            end
        end
        % If there are no crossover frequencies ( Gain < 0 dB ) use dcgain
        if CW == Inf
            CW = 0;
        end
        % Get the Gain and the Phase at the crossover frequency
        [Gain, Phase] = bode(TF,CW);
        RGA = Gain.*inv(Gain)';
end
        

% Get the maximum of each row
sys_pair = zeros(sys_size(2),1);

for output = 1:sys_size(2)
    [val, sys_pair(output,1)] = max(RGA(output,:));
end

% Make the PID for the pairing
C = tf('s'); % Preallocate Controller
K_p = zeros(2,2); % Store Parameter
K_i = zeros(2,2); % Store Parameter
K_d = zeros(2,2); % Sotre Parameter

for output = 1:2
    for input = 1:2
        C(output,input) = tf(0,1);
    end
end

for output = 1:sys_size(2)
    %opts = pidtuneOptions('PhaseMargin',PM(1,output));
    %currentControl = pidtune(TF(output,sys_pair(output,1)),'PI',opts);
    currentControl = AMIGO_Tune(TF(output,sys_pair(output,1)),'PI');
    [kp, ki, kd] = piddata(currentControl);
    K_p(output,sys_pair(output,1)) = kp;
    K_i(output,sys_pair(output,1)) = ki;
    K_d(output,sys_pair(output,1)) = kd;
end

C = pid2(K_p, K_i, K_d, 0, b, 0);
% Close the loop
%CL = feedback(TF*C,eye(2));
end