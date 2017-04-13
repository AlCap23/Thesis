function CL = Decoupling_D(TF,Constrains)

% System Size
sys_size = size(TF);

% Get the Phase Margins
MS = Constrains(1,3:4); % Maximum of the sensitivity
PM = 2*asin(1/2./MS); % Phase Margin is at least 2 arcsin(1/2/Ms)

% Get the disturbance
D = tf('s');

% Get the static Gain
G0 = dcgain(TF);

for outputs = 1:sys_size(1)
    clear CTF
    for inputs = 1:sys_size(2)
        if inputs == outputs
            CTF = tf(0,1);
        else
            CTF = TF(outputs,inputs);
        end
        D(outputs,inputs) = CTF / TF(inputs,inputs);
    end
end

% Make the PID for main diagonal
C = tf('s');

for output = 1:sys_size(2)
    % Tunes for the Phase Margin, which is essentially f(MS)
   opts = pidtuneOptions('PhaseMargin',PM(1,outputs));
   % Tune a PI Controller
   TunedControl = pidtune(TF(output,output),'PI',opts);
end

% Make the closed loop
CL = feedback(TF*(eye(sys_size)-D)*C,eye(sys_size));

end