function C = Decoupling_F(G,Constrains,Method,SetPointWeight)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


% System Size
sys_size = size(G);

% Check Constrains
if size(Constrains) ~= [1 sys_size(2)+sys_size(1)]
    error('Not enough constrains!')
end

% Set set point weight - funny!
if ~exist('SetPointWeight','var')
    b = 0; % Design normal controller
else
    b = SetPointWeight; % Use given Set Point Weight
end

% Check for Method -> Switch between Matlab Algorithm ( Standard ) and
% AMIGO
if ~exist('Method','var')
    Method = 'Standard';
end

% Preprocess Constrains for Gamma_Max Matrix
HMax = ones(sys_size);
% Fill the Antidiagonal with entries corresponding to the max interaction
% Outer Loop / Inputs
for inputs = 1:sys_size(1)
    % Inner Loop / Outputs
    for outputs = 1:sys_size(2)
        if inputs ~= outputs
            HMax(inputs,outputs) = Constrains(inputs);
        end
    end
end

% Get the Maximum Sensititvity in Matrixform
MS = diag(Constrains(sys_size(1)+1:end));

% Preallocate Controller Coefficients
K_P = zeros(sys_size);
K_I = zeros(sys_size);

% Get the TF Data
for inputs= 1: sys_size(1)
   for outputs = 1:sys_size(2)
       if length(G(inputs,outputs).Denominator{:,:}) >= 2
           T(inputs,outputs) = G(inputs,outputs).Denominator{:,:}(end-1)/G(inputs,outputs).Denominator{:,:}(end);
       else
           T(inputs,outputs) = 0;
       end
       if ~isempty(G(inputs,outputs).IODelay)
           L(inputs,outputs) = G(inputs,outputs).IODelay;
       else
           L(inputs,outputs) = 0
       end
       K(inputs,outputs) = dcgain(G(inputs,outputs));
   end
end

% Get the Static Decoupler D = I + S
S = zeros(sys_size);

for inputs= 1:sys_size(1)
    for outputs = 1:sys_size(2)
        if inputs ~= outputs
            S(inputs,outputs) = -K(inputs,outputs)/K(inputs,inputs);
        end
    end
end

% Get the Gamma Matrix which is the First Order Linearization
Gamma = zeros(sys_size);
for inputs = 1:sys_size(1)
    for outputs = 1:sys_size(2)
        if inputs ~= outputs
            Gamma(inputs,outputs) = K(inputs,outputs)*((L(inputs,inputs)+T(inputs,inputs)) - (L(inputs,outputs)+T(inputs,outputs)));
        end
    end
end

% Design the Diagonal Controller
for inputs = 1:sys_size(1)
    for outputs = 1:sys_size(2)
        if inputs == outputs
            switch Method
                case 'Standard'
                    C = pidtune(G(inputs,outputs),'PI');
                    [K_P(inputs,outputs), K_I(inputs,outputs)] = piddata(C);
                case 'AMIGO'
                    C = AMIGO_Tune(G(inputs,outputs),'PI');
                    [K_P(inputs,outputs), K_I(inputs,outputs)] = piddata(C);
            end
        end
    end
end

%% Detuning
% Aström 2001 - Max KI
K_IMax = 1/det(MS)*inv(Gamma)*HMax;
% Aström 2006 - Max KI
%K_IMax = 1/1.2*inv(Gamma)*inv(MS)*HMax;
for inputs = 1:sys_size(1)
    for outputs = 1:sys_size(2)
        if inputs == outputs
            counter = 0;
            while abs(K_I(inputs,outputs)) > abs(K_IMax(inputs,outputs))
                % Leave at certain iteration Counter
                if counter > 1e2
                    break
                end
                C = pid(K_P(inputs,outputs), K_I(inputs,outputs));
                C = AMIGO_Detune(C,G(inputs,outputs));
                [K_P(inputs,outputs), K_I(inputs,outputs)] = piddata(C);
                counter = counter+1;
            end
        end
    end
end

% Use the Splitter
K_P = K_P + S*K_P;
K_I = K_I + S*K_I;
            
C = pid2(K_P,K_I,0,0,b,0);
end

