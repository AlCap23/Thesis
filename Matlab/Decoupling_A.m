function CL = Decoupling_A(TF,Constrains)

% System Size
sys_size = size(TF);

% Constrains for 
k = Constrains(1,1:2); % Interaction for the loops to each other
MS = Constrains(1,3:4); % Maximum of the sensitivity
PM = 2*asin(1/2./MS); % Phase Margin is at least 2 arcsin(1/2/Ms)
MProd = prod(MS); % Sensitivy Product


% Get the Decoupler D
D = inv(dcgain(TF));

% Get the new system Q
Q = TF*D;


% Get the small signal constants k12( influence of input 2 on output 1 )
% and k21
if Q(1,2).Numerator{1,1}(end-1) > 0
    k12 = Q(1,2).Numerator{1,1}(end-1) / Q(1,2).Denominator{1,1}(end);
else
    k12 = 0;
end

if Q(2,1).Numerator{1,1}(end) > 0
    k21= Q(2,1).Numerator{1,1}(end-1) / Q(2,1).Denominator{1,1}(end);
else 
    k21 = 0;
end

% Interaction from Q
kc = [k12, k21];

% Assume set point weighting and make controller
K_p = tf('s'); % proportional gain
K_i = tf('s'); % integral gain

for outputs = 1:sys_size(2)
   opts = pidtuneOptions('PhaseMargin',PM(1,outputs));
   TunedControl = pidtune(Q(outputs,outputs),'PI',opts);
   [kP,kI] = piddata(TunedControl);
   %C(outputs,outputs) = pid2(kP,kI)
   % Check for stability
   if abs(kI) - abs(k(1,sys_size(2))) / abs(kc(1,sys_size(2)-outputs+1)*MProd) > 1e-5
       % If the Condition is not met, scale kI down and calculate kP via
       % Pole Placement 
       kI = k(1,sys_size(2)) / abs(kc(1,sys_size(2)-outputs+1)*MProd);
       % From Pole Placement, Aström Häggalund, PID Control p174
       Damping = 1.0; % Assume high damping
       omega = sqrt(abs(kI*Q(outputs,outputs).Numerator{1,1}(end)*Q(outputs,outputs).Denominator{1,1}(end-1)));
       % Hier noch gute Frage: Was ist k für ein Zählerpolynom? Annahme ist
       % hier konstante, d.h. s -> 0
       kP = (2*Damping*omega*Q(outputs,outputs).Denominator{1,1}(end-1) -1)/Q(outputs,outputs).Numerator{1,1}(end);
       K_p(outputs,outputs) = pid(kP);
       K_i(outputs,outputs) = pid(0,kI);
   else
       K_p(outputs,outputs) = pid(kP);
       K_i(outputs,outputs) = pid(0,kI);
   end
end
K_p;
K_i;
b = 0;
% Closing the loop for set point weight b = 0
CL = feedback(TF*D*(K_i+b*K_p),eye(2));
end

