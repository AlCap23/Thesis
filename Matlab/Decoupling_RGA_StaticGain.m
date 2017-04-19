function CL = Decoupling_RGA_StaticGain(TF,Constrains)

% System Size
sys_size = size(TF);

% Constrains for 
k = Constrains(1,1:2); % Interaction for the loops to each other
MS = Constrains(1,3:4); % Maximum of the sensitivity
PM = 2*asin(1/2./MS); % Phase Margin is at least 2 arcsin(1/2/Ms)
MProd = prod(MS); % Sensitivy Product

% Make a RGA
RGA = dcgain(TF).*inv(dcgain(TF))';

% Get the maximum of each row
sys_pair = zeros(sys_size(2),1);
for output = 1:sys_size(2)
    [val, sys_pair(output,1)] = max(RGA(output,:));
end

% Make the PID for the pairing
C = tf('s');
for output = 1:sys_size(2)
    opts = pidtuneOptions('PhaseMargin',PM(1,output));
    C(output,sys_pair(output,1)) = pidtune(TF(output,sys_pair(output,1)),'PI',opts);
end
% Close the loop
CL = feedback(TF*C,eye(2));
end