function C = Decoupling_FOTD(TF,Constrains,Method,SetPointWeight)
%% Decouples a MIMO System based on
%
%   Design of decoupled PID controllers for MIMO systems
%   by Aström, Johansson and Wang
%   http://ieeexplore.ieee.org/abstract/document/946038/
%

%% Changelog
%   2017/05/02  Created
%   2017/05/03  Corrected wrong interaction
%               Added SetPoint Weight
%               Corrected wrong Code for Detune
%% Inputs
%  TF - a Transfer Function (MIMO) with 2 inputs and 2 outputs
%  Contrains - an array with constrains for the development of the
%  controller
%  Method - Tuning and Detuning Rules: 
%       'Standard' uses a straight approach for the tuning / detuning of the
%       controller
%       'AMIGO' uses the Amigo tuning / detuning rules in an iterative fashion
%  SetPointWeight - a scalar Set Point Weight of the Controller
%% Output
%  C - A 2-Dof Controller with Set Point Weight as a Matrix of System Size
%% Code

% System Size
sys_size = size(TF);
% Check if size is 2x2
if sys_size ~= [2 2]
    error('The size of the system is not 2x2! Unable to handle system')
end

% Check for constrains
if size(Constrains) ~= [1 4]
    error('Give constrains with [k1 k2 MS1 MS2]')
end

% Preprocess constrains for the system
k = Constrains(1,1:2); % Interaction for the loops to each other
MS = Constrains(1,3:4); % Maximum of the sensitivity
MProd = prod(MS); % Sensitivy Product

% Set set point weight
if ~exist('SetPointWeight','var')
    b = 0; % Design normal controller
else
    b = SetPointWeight; % Use given Set Point Weight
end

% Get the parameters
KV = zeros(sys_size); % Process Gain
TV = zeros(sys_size); % Process Time Constant
LV = zeros(sys_size); % Process Delay
for Outputs = 1:sys_size(1)
   for Inputs = 1:sys_size(2)
       KV(Outputs,Inputs) = dcgain(TF(Outputs,Inputs)); % Gain
       if length(TF(Outputs,Inputs).Denominator{:,:}) > 1
           TV(Outputs,Inputs) = TF(Outputs,Inputs).Denominator{:,:}(end-1)/TF(Outputs,Inputs).Denominator{:,:}(end); % Time Constant
       else
           TV(Outputs,Inputs) = 0;
       end
       LV(Outputs,Inputs) = TF(Outputs,Inputs).IODelay; % Delay
       
   end
end

% Get the interaction 
k12 = KV(1,1)*KV(1,2) / (KV(1,1)*KV(1,2)-KV(2,1)*KV(2,2)) * (LV(1,1)+TV(1,1)-LV(1,2)-TV(1,2));
k21 = KV(2,1)*KV(2,2) / (KV(1,1)*KV(1,2)-KV(2,1)*KV(2,2)) * (LV(2,2)+TV(2,2)-LV(2,1)-TV(2,1));
kc = [k12,k21];

% Check for Method
% Apply Matlab tuning if no method is selected
if ~exist('Method','var')
    Method = 'Standard';
end


C = tf('s');% Create empty controller
for inputs = 1:sys_size(1)
    for outputs = 2:sys_size(2)
        C(inputs,outputs) = tf(0,1);
    end
end
K_p = zeros(2,2); % proportional gain storage
K_i = zeros(2,2); % integral gain storage

switch Method
    case 'Standard'
        % Get the robustness constrain
        PM = 2*asin(1/2./MS);% Phase Margin is at least 2 arcsin(1/2/Ms)
        % Loop over the outputs to design a Controller
        for outputs = 1:sys_size(2)
            % Tune for Phase Margin -> Robustness
            opts = pidtuneOptions('PhaseMargin',PM(1,outputs));
            % Tune a Controller
            C(outputs,outputs) = pidtune(TF(outputs,outputs),'PI',opts);
        end
        for outputs = 1:sys_size(2)          
            % Check for interaction -> if not met, use different parameter
            
            %% Difference to Astr�m for coupling!
            % Get gamma' which is gamma / (K_Input * (T_Output+L_Output -T_Input-L_Input))
            input = sys_size(2)-outputs+1; % (Number of inputs) - (current output) +1 = complementary input
            % Since we have to detune the complementary controller
            TunedControl = C(input,input);
            % Get the parameter
            [kP,kI] = piddata(C(input,input));
            c1 = -1*abs(k(1,outputs)) / abs(MProd); % Normal Interaction gamma 
            c1 = c1 / (KV(outputs,input)*(TV(outputs,outputs)+LV(outputs,outputs) - TV(outputs,input)-LV(outputs,input))); % Modified Interaction gamma'
            %% Normal algorithm
            
            % Set Point Weight b > 0:
            % Can be simplified by using the same formula for every check. Do
            % another check nested inside the first
            if b > 0
                % If the condition is not met, calculate kI and kP
                % Calculate the terms
                Damping = 1; % Assume High Damping
                T = TV(input,input); % Time Constant of Process
                K = KV(input,input); % Process Gain
                % Calculate ki from quadratic equation for sqrt(ki) based on Pole
                % Placement -> See PDF Set Point Weight
                a1 = 1+2*Damping*b;
                b1 = -1*sqrt(b^2/(T*K) );
                
                % If the Condition is not met, set kI and kP according to
                % condition
                if abs(a1*kI+b1*sqrt(kI)) - abs(c1) > 1e-5
                    % Check Discriminant
                    if b1^2 -4*a1*c1 > 1e-2
                        kI = min([( -b1 + sqrt( b1^2 -4*a1*c1 ) ) / (2*a1) ,( -b1 - sqrt( b1^2 -4*a1*c1 ) ) / (2*a1)]);
                        kI = kI^2;
                    else
                        kI = - abs(c1)
                    end
                    kP = (2*Damping*T*sqrt(kI*K/T)-1)/K;
                end
                % Store Controller Parameter
                K_p(input,input) = kP
                K_i(input,input) = kI
            % Check for setpoint weight is zero:
            
            elseif abs(kI) - abs(c1) > 1e-5
                % If the Condition is not met, scale kI down and calculate kP via
                % Pole Placement
                kI = - c1;
                % From Pole Placement, Astr�m H�ggalund, PID Control p.174
                Damping = 1.0; % Assume high damping
                omega = sqrt(abs(kI*KV(input,input)*TV(input,input))); % We could apply omega = omega*L to get an upper bound
                % Assume the small signal equation / Taylor series approximation
                kP = (2*Damping*omega*TV(input,input)-1)/KV(input,input);
                % Get the parameter
                K_p(input,input) =kP;
                K_i(input,input) = kI;
                % Use original tuned parameter
            else
                K_p(input,input) =kP;
                K_i(input,input) = kI;
            end
        end
    case 'AMIGO'
        % Loop over the outputs to design a Controller
        for outputs = 1:sys_size(2)
            % Get an AMIGO Rule tuned controller
            C(outputs,outputs) = AMIGO_Tune(TF(outputs,outputs),'PI');
        end
        
        for outputs = 1:sys_size(2)
            %% Difference to Astr�m for coupling!
            % Get gamma' which is gamma / (K_Input * (T_Output+L_Output -T_Input-L_Input))
            input = sys_size(2)-outputs+1; % (Number of inputs) - (current output) +1 = complementary input
            % Since we have to detune the complementary controller
            TunedControl = C(input,input);
            % Get the parameter
            [kP,kI] = piddata(C(input,input));
            % We know the Sensitivity of Q is equivalent to S_ii ( see
            % PDF ). Calculate the Factor
            % First Loop Correction
            g1 = KV(1,2)*KV(2,1) / ( KV(1,1) * KV(2,2) ) * (TV(1,2)-TV(1,1))/TV(1,1);
            g1 = 0;
            % Second Loop Correction
            g2 = KV(1,2)*KV(2,1) / ( KV(1,1) * KV(2,2) ) * (TV(2,1)-TV(2,2))/TV(2,2);
            g2 = 0;
            c1 = 1*abs(k(1,outputs))*(1-g1)*(1-g2) / abs(MProd); % Normal Interaction gamma 
            c1 = c1 / (KV(outputs,input)*(TV(outputs,outputs)+LV(outputs,outputs) - TV(outputs,input)-LV(outputs,input))); % Modified Interaction gamma'
            
            %% Normal algorithm
            
            % Set Point Weight b > 0:
            % Can be simplified by using the same formula for every check. Do
            % another check nested inside the first
            if b > 0
                % If the condition is not met, calculate kI and kP
                % Calculate the terms
                Damping = 1; % Assume High Damping
                T = TV(input,input); % Time Constant of Process
                K = KV(input,input); % Process Gain
                % Calculate ki from quadratic equation for sqrt(ki) based on Pole
                % Placement
                a1 = 1+2*Damping*b; % Coefficient of x^2 
                b1 = -1*sqrt(b^2/(T*K) ); % Coefficient of x
                % Check the decoupling condition iteratively
                % Iteration of the Detuning
                counter = 0; % counter for break
                while abs(a1*kI+b1*sqrt(kI)) - abs(c1) > 1e-5
                    if counter > 1e5
                        break
                    end
                    TunedControl = AMIGO_Detune(TunedControl,TF(input,input));
                    % Get the parameter
                    [kP,kI] = piddata(TunedControl);
                    % Add a count
                    counter = counter+1;
                end
                % Get the parameter
                [kP,kI] = piddata(TunedControl); 
                % Make a PI Controller
                K_p(input,input) = kP;
                K_i(input,input) = kI;
                
                % Check for setpoint weight is zero:
            elseif - abs(c1) + abs(kI)  > 1e-5
                % If the Condition is not met, scale kI down and calculate kP via
                % Iteration of the Detuning
                counter = 0; % counter for break
                while - abs(c1) + abs(kI) > 1e-5
                    if counter > 1e5
                        break
                    end
                    
                    TunedControl = AMIGO_Detune(TunedControl,TF(input,input));
                    % Get the parameter
                    [kP,kI] = piddata(TunedControl);
                    % Add a count
                    counter = counter+1;
                end
                % Get the parameter
                [kP,kI] = piddata(TunedControl); 
                % Make a PI Controller
                K_p(input,input) = kP;
                K_i(input,input) = kI;
                % Use original tuned parameter
            else
                K_p(input,input) = kP;
                K_i(input,input) = kI;
            end
        end
end
% Coupling Rating -> We do not want the pressure to have full influence on Temp 
a = 1;
% Create the controller on the minor diagonal
K_p(1,2) = -1*KV(1,2)/KV(1,1)*K_p(2,2);
K_p(2,1) = -1*KV(2,1)/KV(2,2)*K_p(1,1);

K_i(1,2) = -1*KV(1,2)/KV(1,1)*K_i(2,2);
K_i(2,1) = -1*KV(2,1)/KV(2,2)*K_i(1,1);

% Create a PID Controller with Set Point Weight
C = pid2(K_p,K_i,0,0,b,0);
end

