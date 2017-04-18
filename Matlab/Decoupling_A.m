function CL = Decoupling_A(TF,Constrains)
%% Decouples a MIMO System based on
%
%   Design of decoupled PID controllers for MIMO systems
%   by Astr�m, Johansson and Wang
%   http://ieeexplore.ieee.org/abstract/document/946038/
%

%% Changelog
%  2017/04/18 added corrected set point weight
%% Inputs
%  TF - a Transfer Function (MIMO) with 2 inputs and 2 outputs
%  Contrains - an array with constrains for the development of the
%  controller

%% Output
%  CL - the closed Loop transfer function

%% Code

% Inital Commit on 2017/04/12
% To Do : Add set point weight b

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

% Set omega and b
omega = 200; % Crossover at 200 rad/s
b = 0.2; % Set point weight

% Get the Decoupler D
D = inv(dcgain(TF));

% Get the new system Q
Q = TF*D;


% Get the small signal constants k12( influence of input 2 on output 1 )
% and k21
% Check if the Numerator is zero, would mean no coupling
if Q(1,2).Numerator{1,1}(end-1) > 0
    % Approximate te coupling
    k12 = Q(1,2).Numerator{1,1}(end-1) / Q(1,2).Denominator{1,1}(end);
else
    % Set coupling to zero
    k12 = 0;
end

if Q(2,1).Numerator{1,1}(end) > 0
    k21= Q(2,1).Numerator{1,1}(end-1) / Q(2,1).Denominator{1,1}(end);
else 
    k21 = 0;
end

% Interaction from Q
kc = [k12, k21]

% Assume set point weighting and make controller
K_p = tf('s'); % proportional gain
K_i = tf('s'); % integral gain

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
       T = Q(outputs,outputs).Denominator{1,1}(end-1); % Time Constant of Process
       K = Q(outputs,outputs).Numerator{1,1}(end); % Process Gain
       % Calculate ki from quadratic equation for sqrt(ki) based on Pole
       % Placement
       a1 = 1+2*Damping*b 
       b1 = -1*sqrt(b^2/(T*K) )
       c1 = -1*abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd)
       if abs(a1*kI+b1*sqrt(kI)) + c1 > 1e-5
        kI = min([( -b1 + sqrt( b1^2 -4*a1*c1 ) ) / (2*a1) ,( -b1 - sqrt( b1^2 -4*a1*c1 ) ) / (2*a1)])
        kI = kI^2
        kP = (2*Damping*T*sqrt(kI*K/T)-1)/K
       end
       K_p(outputs,outputs) = pid(kP);
       K_i(outputs,outputs) = pid(0,kI);
       
   % Check for setpoint weight is zero:
   elseif abs(kI) - abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd) > 1e-5
       % If the Condition is not met, scale kI down and calculate kP via
       % Pole Placement 
       kI = k(1,sys_size(2)) / abs(kc(1,sys_size(2)-outputs+1)*MProd);
       % From Pole Placement, Astr�m H�ggalund, PID Control p.174
       Damping = 1.0; % Assume high damping
       omega = sqrt(abs(kI*Q(outputs,outputs).Numerator{1,1}(end)*Q(outputs,outputs).Denominator{1,1}(end-1)));
       % Assume the small signal equation / Taylor series approximation
       kP = (2*Damping*omega*Q(outputs,outputs).Denominator{1,1}(end-1) -1)/Q(outputs,outputs).Numerator{1,1}(end);
       % Get the parameter
       K_p(outputs,outputs) = pid(kP);
       K_i(outputs,outputs) = pid(0,kI);
   % Use original tuned parameter
   else
       K_p(outputs,outputs) = pid(kP);
       K_i(outputs,outputs) = pid(0,kI);
   end
end
b = 0;
% Closing the loop for set point weight b = 0
CL = (eye(2)+TF*D*(K_i+K_p)) \ TF*D*(K_i+b*K_p);
end

