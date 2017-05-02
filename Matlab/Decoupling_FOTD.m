function C = Decoupling_FOTD(TF,Constrains)
%% Decouples a MIMO System based on
%
%   Design of decoupled PID controllers for MIMO systems
%   by Aström, Johansson and Wang
%   http://ieeexplore.ieee.org/abstract/document/946038/
%

%% Changelog
%  2017/05/02 Created
%% Inputs
%  TF - a Transfer Function (MIMO) with 2 inputs and 2 outputs
%  Contrains - an array with constrains for the development of the
%  controller
%  Method - Tuning and Detuning Rules: 
%       'Standard' uses a straight approach for the tuning / detuning of the
%       controller
%       'AMIGO' uses the Amigo tuning / detuning rules in an iterative fashion

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
b = 0;

% Get the parameters
K = zeros(4,1); % Process Gain
T = zeros(4,1); % Process Time Constant
L = zeros(4,1); % Process Delay
counter = 1; % Counter
for Outputs = 1:2
   for Inputs = 1:2
       K(counter) = dcgain(TF(Outputs,Inputs)); % Gain
       T(counter) = TF(Outputs,Inputs).Denominator{:,:}(end-1); % Time Constant
       L(counter) = TF(Outputs,Inputs).IODelay; % Delay
       counter = counter+1;
   end
end

% Get the interaction 
%k12 = K(1)*K(2) / (K(1)*K(2)-K(3)*K(4)) * (L(1)+T(1)-L(2)-T(2));
%k12 = K(3)*K(4) / (K(1)*K(2)-K(3)*K(4)) * (L(4)+T(4)-L(3)-T(3));

% Get the controller, assuming main loop dominance
C = tf('s');
% Assume set point weighting and make controller
K_p = zeros(2,2); % proportional gain storage
K_i = zeros(2,2); % integral gain storage
for Outputs = 1:2
    % Make an empty TF
    for Inputs = 1:2
        C(Outputs,Inputs) = tf(0,1);
    end
    C(Outputs,Outputs) = AMIGO_Tune(TF(Outputs,Outputs),'PI');
end

% Test for interaction
% First Loop
gamma = k(1)/MProd;
[kp,ki] = piddata(C(1,1));
counter = 0;
while abs(K(2)*(L(1)+T(1)-L(2)-T(2))*(kp*b+ki))- abs(gamma) >1e-5
    counter
    if counter > 1e4
        break
    end
    C(1,1) = AMIGO_Detune(C(1,1),TF(1,1)); % Detune via AMIGO Rules 
    [kp,ki] = piddata(C(1,1)); % Get new parameter
    counter = counter+1; 

end
K_p(1,1) = kp;
K_i(1,1) = ki;


% Second Loop
gamma = k(2)/MProd;
[kp,ki] = piddata(C(2,2));
counter = 0;
while abs(K(3)*(L(4)+T(4)-L(3)-T(3))*(kp*b+ki))- abs(gamma) > 1e-5
    counter
    if counter > 1e4
        break
    end
    C(2,2) = AMIGO_Detune(C(2,2),TF(2,2)); % Detune via AMIGO Rules 
    [kp,ki] = piddata(C(2,2)); % Get new parameter
    counter = counter+1; 

end

K_p(2,2) = kp;
K_i(2,2) = ki;


% Create the controller on the minor diagonal
K_p(1,2) = -K(2)/K(1)*K_p(2,2);
K_p(2,1) = -K(3)/K(4)*K_p(1,1);

K_i(1,2) = -K(2)/K(1)*K_i(2,2);
K_i(2,1) = -K(3)/K(4)*K_i(1,1);

% Create a PID Controller with Set Point Weight
C = pid2(K_p,K_i,0,0,b,0);

