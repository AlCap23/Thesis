function C = Decoupling_A(TF,Constrains,Method,SetPointWeight)
%% Decouples a MIMO System based on
%
%   Design of decoupled PID controllers for MIMO systems
%   by Astr�m, Johansson and Wang
%   http://ieeexplore.ieee.org/abstract/document/946038/
%

%% Changelog
%  2017/04/18   Added corrected set point weight
%  2017/05/03   Added Set Point Weight as Input Parameter
%               Corrected Code
%               Simplified Input of Transferfunction
%               Added the Approximation for Q as FOTD
% 2017/05/04    Corrected the Approximation for Q as FOTD
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
PM = 2*asin(1/2./MS); % Phase Margin is at least 2 arcsin(1/2/Ms)
MProd = prod(MS); % Sensitivy Product

% Set set point weight
if ~exist('SetPointWeight','var')
    b = 0; % Design normal controller
else
    b = SetPointWeight; % Use given Set Point Weight
end

% Get the Decoupler D
D = inv(dcgain(TF))

% Get the new system Q
if abs(sum(sum(TF.IODelay))) > 0
    Q = pade(TF,6)*D
else
    Q = TF*D
end

% figure()
% step(TF)
% hold on
% grid on
% step(Q)
% legend('G','Q')
% 
% figure()
% bode(TF)
% hold on
% grid on
% bode(Q)
% legend('G','Q')

% Check if the main diagonal elements are of order 1
for output = 1: sys_size(1)
    % Check for TF Order, if one diagonal is of higher Order the process is
    % needed
    if order(Q(output,output)) ~= 1
        
        % The Transferfunctions needed are of the same Output / Row and
        % scaled by the same Column of the D Matrix
        G1 = D(1,output)*TF(output,1);
        G2 = D(2,output)*TF(output,2);
        % Get the Coefficients
        K1 = dcgain(G1);
        T1 = G1.Denominator{:,:}(end-1)/G1.Denominator{:,:}(end);
        L1 = G1.IODelay;
        
        K2 = dcgain(G2);
        T2 = G2.Denominator{:,:}(end-1)/G2.Denominator{:,:}(end);
        L2 = G2.IODelay;
        
        % Set new parameter
        L = max(L1,L2);
        K = K1+K2;
        T = (K1*(T1+L1)+K2*(T2+L2))/K-L;
        
        % Form new transfer function of the main Diagonal
        Q(output,output) = tf(K,[T,1],'IODelay',L);
    end
end

% Get the small signal constants k12( influence of input 2 on output 1 )
% and k21
% Check if the Numerator is zero, would mean no coupling
if ~isempty(Q(1,2).Denominator{:,:}) && length(Q(1,2).Denominator{:,:}) > 1
    k12 = Q(1,2).Numerator{:,:}(end-1) / Q(1,2).Denominator{:,:}(end);
else
    k12 = 0;
end
if ~isempty(Q(2,1).Denominator{:,:}) && length(Q(2,1).Denominator{:,:}) > 1
    k21 = Q(2,1).Numerator{:,:}(end-1) / Q(2,1).Denominator{:,:}(end);
else
    k21 = 0;
end

% Interaction from Q
kc = [k12, k21];

% Assume set point weighting and make controller
K_p = zeros(2,2); % proportional gain storage
K_i = zeros(2,2); % integral gain storage

% Check for Method

if ~exist('Method','var')
    Method = 'Standard';
end

switch Method
    case 'Standard'
        for outputs = 1:sys_size(2)
            % Tunes for the Phase Margin, which is essentially f(MS)
            opts = pidtuneOptions('PhaseMargin',PM(1,outputs));
            % Tune a PI Controller
            TunedControl = pidtune(Q(outputs,outputs),'PI',opts);
            % Get the parameter
            [kP,kI] = piddata(TunedControl);
            
            % Check for interaction -> if not met, use different parameter
            
            % Set Point Weight b > 0:
            % Can be simplified by using the same formula for every check. Do
            % another check nested inside the first
            if b > 0
                % If the condition is not met, calculate kI and kP
                % Calculate the terms
                Damping = 1; % Assume High Damping
                T = Q(outputs,outputs).Denominator{1,1}(end-1)/Q(outputs,outputs).Denominator{1,1}(end); % Time Constant of Process
                K = dcgain(Q(outputs,outputs)); % Process Gain
                % Calculate ki from quadratic equation for sqrt(ki) based on Pole
                % Placement
                a1 = 1+2*Damping*b;
                b1 = -1*sqrt(b^2/(T*K) );
                c1 = -1*abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd);
                % If the Condition is not met, set kI and kP according to
                % condition
                if abs(a1*kI+b1*sqrt(kI)) + c1 > 1e-5
                    % Check Discriminant
                    if b1^2 -4*a1*c1 > 1e-2
                        kI = min([( -b1 + sqrt( b1^2 -4*a1*c1 ) ) / (2*a1) ,( -b1 - sqrt( b1^2 -4*a1*c1 ) ) / (2*a1)]);
                        kI = kI^2;
                    else
                        kI = c1
                    end
                    kP = (2*Damping*T*sqrt(kI*K/T)-1)/K;
                end
                K_p(outputs,outputs) = kP;
                K_i(outputs,outputs) = kI;
                
                % Check for setpoint weight is zero:
            elseif abs(kI) - abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd) > 1e-2
                % If the Condition is not met, scale kI down and calculate kP via
                % Pole Placement
                kI = k(1,sys_size(2)) / abs(kc(1,sys_size(2)-outputs+1)*MProd);
                % From Pole Placement, Astr�m H�ggalund, PID Control p.174
                Damping = 1.0; % Assume high damping
                omega = sqrt(abs(kI*dcgain(Q(outputs,outputs))*Q(outputs,outputs).Denominator{1,1}(end-1)/Q(outputs,outputs).Denominator{1,1}(end)));
                % Assume the small signal equation / Taylor series approximation
                kP = (2*Damping*omega*Q(outputs,outputs).Denominator{1,1}(end-1)/Q(outputs,outputs).Denominator{1,1}(end) -1)/dcgain(Q(outputs,outputs));
                % Get the parameter
                K_p(outputs,outputs) =kP;
                K_i(outputs,outputs) = kI;
                % Use original tuned parameter
            else
                K_p(outputs,outputs) = kP;
                K_i(outputs,outputs) = kI;
            end
        end
    case 'AMIGO'
        for outputs = 1:sys_size(2)
            % Get a AMIGO Rule tuned controller
            TunedControl = AMIGO_Tune(Q(outputs,outputs),'PI');
            % Get the parameter
            [kP,kI] = piddata(TunedControl);
            
            % Check for interaction -> if not met, use different parameter
            
            % Set Point Weight b > 0:
            % Can be simplified by using the same formula for every check. Do
            % another check nested inside the first
            if b > 0
                % If the condition is not met, calculate kI and kP
                % Calculate the terms
                Damping = 1; % Assume High Damping
                T = Q(outputs,outputs).Denominator{1,1}(end-1)/Q(outputs,outputs).Denominator{1,1}(end); % Time Constant of Process
                K = dcgain(Q(outputs,outputs)); % Process Gain
                % Calculate ki from quadratic equation for sqrt(ki) based on Pole
                % Placement
                a1 = 1+2*Damping*b;
                b1 = -1*sqrt(b^2/(T*K) );
                c1 = -1*abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd);
                % Check the decoupling condition iteratively
                % Iteration of the Detuning
                counter = 0; % counter for break
                while abs(a1*kI+b1*sqrt(kI)) + c1 > 1e-5
                    if counter > 1e4
                        break
                    end
                    TunedControl = AMIGO_Detune(TunedControl,Q(outputs,outputs));
                    % Get the parameter
                    [kP,kI] = piddata(TunedControl);
                    % Add a count
                    counter = counter+1;
                end
                % Get the parameter
                [kP,kI] = piddata(TunedControl); 
                % Make a PI Controller
                K_p(outputs,outputs) = kP;
                K_i(outputs,outputs) = kI;
                
                % Check for setpoint weight is zero:
            elseif + abs(kI) - abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd) > 1e-5
                % If the Condition is not met, scale kI down and calculate kP via
                % Iteration of the Detuning
                counter = 0; % counter for break
                while + abs(kI) - abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd) > 1e-5
                    if counter > 1e4
                        break
                    end
                    
                    TunedControl = AMIGO_Detune(TunedControl,Q(outputs,outputs));
                    % Get the parameter
                    [kP,kI] = piddata(TunedControl);
                    % Add a count
                    counter = counter+1;
                end
                % Get the parameter
                [kP,kI] = piddata(TunedControl); 
                % Make a PI Controller
                K_p(outputs,outputs) = kP;
                K_i(outputs,outputs) = kI;
                % Use original tuned parameter
            else
                K_p(outputs,outputs) = kP;
                K_i(outputs,outputs) = kI;
            end
        end
end 
% Create a PID Controller with Set Point Weight
K_p = D*K_p;
K_i = D*K_i;
C = pid2(K_p,K_i,0,0,b,0);

% Closing the loop for set point weight b = 0
%CL = (eye(2)+TF*D*(K_i+K_p)) \ TF*D*(K_i+b*K_p);
end

