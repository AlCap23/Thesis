function CL = Decoupling_D(TF)

% System Size
sys_size = size(TF);
% Get the disturbance
D = tf('s');

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
    C(output,output) = pidtune(TF(output,output),'PI');
end
% Make the closed loop
CL = feedback(TF*(eye(sys_size)-D)*C,eye(sys_size));

end